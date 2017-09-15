############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Contains the components neccessary to detect objects at multiple scales
"""

from __future__ import absolute_import, division, print_function, unicode_literals

import os

import cv2
import numpy as np
from collections import namedtuple

from .model import MODELS
from .data import *

# default IoU threshold for non-maximum suppression
IOU_THRESH = .5
# iou threshold to use for the 3rd stage of the cascade
LAST_STAGE_IOU_THRESH = .2
# minimum score for 12-net to classify a region as positive
NET_12_THRESH = .006
# minimum score for 24-net to classify a region as positive
NET_24_THRESH = .044
# minimum score for 48-net to classify a region as positive
NET_48_THRESH = .5

# default number of pyramid levels to use for detect_multiscale
NUM_PYRAMID_LEVELS = 5
# amount to shrink the previous pyramid level's image by
PYRAMID_DOWNSCALE = 1.25

# default number of calibration steps to use for the fast_detect function
NUM_CALIBRATION_STEPS = 1

# folder for the face detector files
FACE_BASE_FOLDER = 'face'
# folder for the roomba detector files
ROOMA_BASE_FOLDER = 'roomba'

# base folder field name for ObjectTypes
BASE_FOLDER_FIELD_NAME = 'base_folder'
# fast detect parameters field name for ObjectTypes
FAST_DETECT_PARAMS_FIELD_NAME = 'fast_detect_params'

# default bounding box color (green)
DEFAULT_BOUNDING_BOX_COLOR = (0, 255, 0)
# default bounding box thickness
DEFAULT_BOUNDING_BOX_THICKNESS = 3

# lower bound for the hue of the red roomba's flap
RED_ROOMBA_HUE_MIN = 0
# upper bound for the hue of the red roomba's flap
RED_ROOMBA_HUE_MAX = 179
# lower bound for the saturation of the red roomba's flap
RED_ROOMBA_SATURATION_MIN = 130
# upper bound for the saturation of the red roomba's flap
RED_ROOMBA_SATURATION_MAX = 255
# lower bound for the hue of the green roomba's flap
GREEN_ROOMBA_HUE_MIN = 46
# upper bound for the hue of the green roomba's flap
GREEN_ROOMBA_HUE_MAX = 66
# lower bound for the saturation of the green roomba's flap
GREEN_ROOMBA_SATURATION_MIN = 60
# upper bound for the saturation of the green roomba's flap
GREEN_ROOMBA_SATURATION_MAX = 255
# minimum area for a region to have a chance at being considered a roomba
MIN_ROOMBA_AREA = 50

def iou(boxes, box, area=None):
    """
    Computes the intersection over union ratio between box and each box in boxes.
    
    Parameters
    ----------
    boxes: numpy.ndarray
           Box array. Each row should have the format (x_min, y_min, x_max, y_max)
    box: numpy.array
         The box that we measure intersection over union relative to. Should have the same format as boxes. 
    area: numpy.array, optional
          Area of each box in boxes
    
    Returns
    -------
    out: numpy.array
    An array containing the calculated intersection over union values between box and each box in boxes
    """

    int_x1, int_y1, int_x2, int_y2 = ((np.maximum if i < 2 else np.minimum)(boxes[:, i], box[i]) for i in np.arange(boxes.shape[1]))
    area = (boxes[:,2]-boxes[:,0])*(boxes[:,3]-boxes[:, 1]) if not area else area
    int_area = (np.maximum(0,int_x2-int_x1))*(np.maximum(0,int_y2-int_y1))
    union_area = area+(box[2]-box[0])*(box[3]-box[1])-int_area
    int_over_union = int_area/union_area
    return int_over_union

def nms(boxes, predictions, iou_thresh=IOU_THRESH):
    """
    Non-maximum suppression algorithm which suppresses overlapping boxes that the detector is less confident in.
    
    Parameters
    ----------
    boxes : numpy.ndarray
           Box array. Each row should have the format (x_min, y_min, x_max, y_max)
    predictions : numpy.array
           Array of probabilities corresponding to the likelihood that a given box contains the target object.
    iou_thresh : float, optional
           The minimum intersection over union value needed for two boxes to be overlapping, default is `IOU_THRESH`.
    
    Returns
    -------
    out: numpy.array, numpy.array
    Returns the boxes that weren't suppressed and the indices of the boxes that were kept
    """

    idxs = np.argsort(predictions)
    picked = []

    while len(idxs) > 0:
        pick = boxes[idxs[-1]]
        picked.append(idxs[-1])

        int_over_union = iou(boxes[idxs[:-1]], pick)
        idxs = np.delete(idxs, np.concatenate(([len(idxs)-1],np.where(int_over_union > iou_thresh)[0])))
    
    return boxes[picked], picked

def _local_nms(boxes, predictions, pyr_idxs, iou_thresh=IOU_THRESH):
    """
    Applies non-maximum suppression locally to each image pyramid level
    
    Parameters
    ----------
    boxes : numpy.ndarray
           Box array. Each row should have the format (x_min, y_min, x_max, y_max)
    predictions : numpy.array
           Array of probabilities corresponding to the likelihood that a given box contains the target object.
    pyr_idxs : sequence of ints
           A sequence of starting indices for each pyramid level. The length of the `boxes` param must be the final entry.
    iou_thresh : float, optional
           The minimum intersection over union value needed for two boxes to be overlapping, default is `IOU_THRESH`.
    
    Returns
    -------
    out: numpy.array, numpy.array, list
    Returns the `boxes` that weren't suppressed, the indices of the `boxes` kept, and the `pyr_idxs` sequence post-suppression.

    See Also
    ----------
    `detect._nms`
    """

    suppressed_boxes = np.zeros(((0,)+boxes.shape[1:]))
    prev_idx = 0
    new_pyr_idxs = []
    picked = []

    for cur_idx in pyr_idxs:
        local_suppressed_boxes, local_picked = nms(boxes[prev_idx:cur_idx], predictions[prev_idx:cur_idx], iou_thresh)
        suppressed_boxes = np.vstack((suppressed_boxes, local_suppressed_boxes))
        picked.extend(local_picked)
        prev_idx = cur_idx
        new_pyr_idxs.append(suppressed_boxes.shape[0])

    return suppressed_boxes, picked, new_pyr_idxs

def _get_detection_windows(img, scale, min_object_scale=MIN_OBJECT_SCALE, num_pyramid_levels=NUM_PYRAMID_LEVELS, pyramid_downscale=PYRAMID_DOWNSCALE):
    """
    Sliding window and image pyramid detection window generator
    
    Parameters
    ----------
    img : numpy.ndarray
           Image to yield detection windows from
    scale : int
           Generate detection windows with dimensions `scale`x`scale`
    min_object_scale : int, optional
           `min_object_scale`x`min_object_scale` is the smallest region for which the detector will be effective. However,
           good results may still be acheived with slightly smaller regions due to bounding box calibration. Default is `MIN_OBJECT_SCALE`.
    num_pyramid_levels : int, optional
           The maximum number of levels to use for the image pyramid generated from the `img` param. If `num_pyramid_levels` is larger
           than the maximum level available for `img` with `pyramid_downscale`, then the maximum possible level is used instead.
           Default is `NUM_PYRAMID_LEVELS`.
    pyramid_downscale : float, optional
           The amount to shrink the previous image pyramid level's image by, default is `PYRAMID_DOWNSCALE`
    
    Returns
    -------
    out: generator
    The first value yielded by the generator will be the number of detection windows present. After that, the generator can be called
    in a loop to retrieve the pyramid level, minimum x value, minimum y value, maximum x value, maximum y value, and cropped out detection
    window until the entire image has been scanned at all the scales specified
    """

    num_pyramid_levels = min(int(np.log(min(img.shape[:2])/min_object_scale)/np.log(pyramid_downscale)), num_pyramid_levels)
    scales = [scale/(min_object_scale*pyramid_downscale**i) for i in np.arange(num_pyramid_levels)]
    resized = [cv2.resize(img, None, fx=scale, fy=scale) for scale in scales]
    yield sum((num_detection_windows_along_axis(img.shape[0])*num_detection_windows_along_axis(img.shape[1]) for img in resized))

    for pyr_level, img in enumerate(resized):
        for y_idx in np.arange(num_detection_windows_along_axis(img.shape[0])):
            for x_idx in np.arange(num_detection_windows_along_axis(img.shape[1])):
                x_min, y_min, x_max, y_max = (x_idx*OFFSET, y_idx*OFFSET, x_idx*OFFSET+scale, y_idx*OFFSET+scale)
                yield (pyr_level, x_min, y_min, x_max, y_max, img[y_min:y_max, x_min:x_max])

def _calibrate_coordinates(coords, calib_predictions):
    """
    Calibrate the collection of bounding boxes contained within `coords` to better fit any target object contained within
    
    Parameters
    ----------
    coords : numpy.ndarray
           Box coordinate array. Each row should have the format (x_min, y_min, x_max, y_max)
    calib_predictions : numpy.array
           Array of probabilities indicating the likelihood that any one transformation should be used for calibration of the
           corresponding coordinates
    
    Returns
    -------
    out: numpy.ndarray
    Returns an array of calibrated bounding box coordinates
    """

    calib_transformations = CALIB_PATTERNS_ARR[np.argmax(calib_predictions, axis=1)]
    sn = calib_transformations[:, 0]
    dimensions = (coords[:, 2:]-coords[:, :2])/sn[:, None]
    coords[:, :2] -= dimensions*calib_transformations[:, 1:]
    coords[:, 2:] = coords[:, :2] + dimensions
    return coords

def _get_network_inputs(img, cur_scale, coords): 
    """
    Extract neural network inputs from `img` at the locations specified by `coords`
    
    Parameters
    ----------
    img : numpy.ndarray
           Image to extract network inputs from
    cur_scale : int
           Network inputs will be resized to be `cur_scale`x`cur_scale`
    coords: numpy.ndarray
           Locations in the image to crop out and resize

    Returns
    -------
    out: numpy.ndarray
    Returns an array of unnormalized network inputs
    """

    inputs = np.ones((len(coords), cur_scale, cur_scale, 3), dtype=np.uint8)

    for i, (x_min, y_min, x_max, y_max) in enumerate(coords.astype(np.int32)):
        x_min, y_min, w, h = squash_coords(img, x_min, y_min, x_max-x_min, y_max-y_min)
        inputs[i] = cv2.resize(img[y_min:y_min+h, x_min:x_min+w], (cur_scale, cur_scale))

    return inputs

class CNNCascadeObjectDetector():
    """
    Class which provides a high-level interface for the object detection method described by [1].

    Parameters
    ----------
    dataset_manager: dict
        Parameters for the DatasetManager which manages access to the data used by the CNN cascade.
    stage_one_thresh: int, optional
        Minimum score needed for the stage one classifier to classify a region as positive, defaults to `NET_12_THRESH`.
    stage_two_thresh: int, optional
        Minimum score needed for the stage two classifier to classify a region as positive, defaults to `NET_24_THRESH`.
    stage_three_thresh: int, optional
        Minimum score needed for the stage three classifier to classify a region as positive, defaults to `NET_48_THRESH`.
    last_stage_iou_thresh: int, optional
        IoU threshold to use in non-maximum suppression at the end of the last cascade stage, defaults to `LAST_STAGE_IOU_THRESH`.
    max_stage_idx: int, optional
        The index of the highest cascade stage the detector should use, default behavior is to use all stages of the cascade.
    min_object_scale: int, optional
        `min_object_scale`x`min_object_scale` is the smallest region for which the detector will be effective. However,
         good results may still be acheived with slightly smaller regions due to bounding box calibration. Default is `MIN_OBJECT_SCALE`.
    num_pyramid_levels : int, optional
           The maximum number of levels to use for the image pyramid generated from the `img` param. If `num_pyramid_levels` is larger
           than the maximum level available for `img` with `pyramid_downscale`, then the maximum possible level is used instead.
           Default is `NUM_PYRAMID_LEVELS`.
    pyramid_downscale : float, optional
           The amount to shrink the previous image pyramid level's image by, default is `PYRAMID_DOWNSCALE`

    References
    ----------
    .. [1] Li, H., Lin, Z., Shen, X., Brandt, J., & Hua, G. (2015). A convolutional neural network cascade for face detection. 
       In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (pp. 5325-5334).
    """

    # cached calibrators and classifiers
    MODELS = [(MODELS[False][stage_idx], MODELS[True][stage_idx]) for stage_idx in np.arange(len(SCALES))]
    # cached input normalizers
    NORMALIZERS = {}

    def __init__(self, dataset_manager_params, **kwargs):
        """
        Prepares the cascade for object detection tasks by loading in any cached data and initializing internal
        attributes.
        """

        self._stage_one_thresh = NET_12_THRESH
        self._stage_two_thresh = NET_24_THRESH
        self._stage_three_thresh = NET_48_THRESH
        self._last_stage_iou_thresh = LAST_STAGE_IOU_THRESH
        self._max_stage_idx = len(SCALES)-1
        self._min_object_scale = MIN_OBJECT_SCALE # minimum scale to scan the image at when using sliding window.
        self._pyramid_downscale = PYRAMID_DOWNSCALE
        self._num_pyramid_levels = NUM_PYRAMID_LEVELS
        self._num_calibration_steps = NUM_CALIBRATION_STEPS

        for attr, value in kwargs.items():
            attr = '_' + attr

            if not hasattr(self, attr):
                raise NameError('Keyword argument "%s" was not found.' % attr[1:])

            setattr(self, attr, value)


        self._thresholds = (self._stage_one_thresh, self._stage_two_thresh, self._stage_three_thresh)
        self._dataset_manager_params = dataset_manager_params
        self._normalizers = self._get_normalizers()

    def _get_normalizers(self):
        """
        Get the cached normalizer list for the selected CNN cascade.

        Returns
        -------
        out: list of tuple
        If normalizers have been cached, this function will return a list that contains each cached stage's classifier normalizer
        and calibrator normalizer together in a tuple object.
        """

        self._hash = ''.join([str(value) for value in self._dataset_manager_params.values()])

        if self._hash not in CNNCascadeObjectDetector.NORMALIZERS:
            CNNCascadeObjectDetector.NORMALIZERS[self._hash] = []
        
        return CNNCascadeObjectDetector.NORMALIZERS[self._hash]

    def _get_cur_scale(self, stage_idx):
        """
        Get the image input scale for the cascade stage corresponding to `stage_idx`.
        
        Parameters
        ----------
        stage_idx : int
            Cascade stage index

        Returns
        -------
        out: int
        Returns the scale of the image inputs for the specfied cascade stage.
        """

        return SCALES[stage_idx][0]

    def _get_models(self, stage_idx):
        """
        Get the models for the cascade stage corresponding to `stage_idx`.
        
        Parameters
        ----------
        stage_idx : int
            Cascade stage index

        Returns
        -------
        out: tuple
        Returns the models in the form (classifier, calibrator).
        """

        return CNNCascadeObjectDetector.MODELS[stage_idx]

    def detect_multiscale(self, img):
        """
        Detects objects at multiple scales
        
        Parameters
        ----------
        img : numpy.ndarray
            Image to detect objects in

        Returns
        -------
        out: numpy.ndarray
        Returns the coordinates for each detected object's bounding box. Each coordinate set is of the form (x_min, y_min, x_max, y_max) 
        """

        classifier_inputs = []
        calibrator_inputs = []

        for stage_idx in np.arange(0, self._max_stage_idx + 1):
            cur_scale = self._get_cur_scale(stage_idx)
            classifier, calibrator = self._get_models(stage_idx)

            if len(self._normalizers) <= stage_idx:
                self._normalizers.append(list())

                for model in (classifier, calibrator):
                    self._normalizers[-1].append(DatasetManager(model, **self._dataset_manager_params).get_normalizer())

            if stage_idx == 0:
                generator_params = {'min_object_scale': self._min_object_scale, 
                                    'num_pyramid_levels': self._num_pyramid_levels, 
                                    'pyramid_downscale': self._pyramid_downscale}
                detection_window_generator = _get_detection_windows(img, cur_scale, **generator_params)
                total_num_detection_windows = next(detection_window_generator)
                detection_windows = np.zeros((total_num_detection_windows, cur_scale, cur_scale, 3))
                coords = np.zeros((total_num_detection_windows, 4))
                pyr_idxs = []
                prev_pyr_level = 0

                for i, (pyr_level, x_min, y_min, x_max, y_max, detection_window) in enumerate(detection_window_generator):
                    detection_windows[i] = detection_window
                    coords[i] = (x_min, y_min, x_max, y_max)
                    coords[i] *= self._pyramid_downscale ** pyr_level

                    if pyr_level != prev_pyr_level:
                        prev_pyr_level = pyr_level
                        pyr_idxs.append(i)

                pyr_idxs.append(len(detection_windows))
                coords *= self._min_object_scale/cur_scale
            else:
                for i in np.arange(0, stage_idx):
                    classifier_inputs[i] = self._normalizers[i][0].preprocess(_get_network_inputs(img, classifier_inputs[i].shape[1], coords).astype(np.float))

            classifier_normalizer, calib_normalizer = self._normalizers[stage_idx]

            detection_windows = detection_windows if stage_idx == 0 else _get_network_inputs(img, cur_scale, coords).astype(np.float)
            classifier_inputs.insert(0 if stage_idx < 2 else stage_idx, classifier_normalizer.preprocess(detection_windows))
            predictions = classifier.predict(classifier_inputs)[:,1]
            pos_detection_indices = np.where(predictions>=self._thresholds[stage_idx])

            calibrator_inputs = [calib_normalizer.preprocess(detection_windows[pos_detection_indices])]
            calib_predictions = calibrator.predict(calibrator_inputs)
            coords = _calibrate_coordinates(coords[pos_detection_indices], calib_predictions)

            if stage_idx == len(SCALES)-1:
                coords, picked = nms(coords, predictions[pos_detection_indices], iou_thresh=self._last_stage_iou_thresh)
            else:
                coords, picked, pyr_idxs = _local_nms(coords, predictions[pos_detection_indices], pyr_idxs)

            if stage_idx == self._max_stage_idx:
                return coords.astype(np.int32, copy=False)

    def fast_detect(self, img, get_detection_windows):
        stage_idx = 0
        num_calib_steps = self._num_calibration_steps
        cur_scale = self._get_cur_scale(stage_idx)
        
        object_proposals, centers = get_detection_windows(img)
        centers = np.asarray(centers)
        total_num_detection_windows = len(object_proposals)
        detection_windows = np.zeros((total_num_detection_windows, cur_scale, cur_scale, 3))
        coords = np.zeros((total_num_detection_windows, 4))

        for i, (x_min, y_min, x_max, y_max) in enumerate(object_proposals):
            x_min, y_min, w, h = squash_coords(img, x_min, y_min, x_max-x_min, y_max-y_min)
            coords[i] = (x_min, y_min, x_min + w, y_min + h)
            detection_windows[i] = cv2.resize(img[y_min:y_min+h, x_min:x_min+w], (cur_scale, cur_scale))

        classifier, calibrator = self._get_models(stage_idx)

        if len(self._normalizers) <= stage_idx:
            self._normalizers.append(list())

            for model in (classifier, calibrator):
                self._normalizers[-1].append(DatasetManager(model, **self._dataset_manager_params).get_normalizer())

        classifier_normalizer, calib_normalizer = self._normalizers[stage_idx]

        predictions = classifier.predict([classifier_normalizer.preprocess(detection_windows)])[:,1]
        pos_detection_indices = np.where(predictions>=self._thresholds[stage_idx])
        coords, centers = coords[pos_detection_indices], centers[pos_detection_indices]
        predictions, detection_windows = predictions[pos_detection_indices], detection_windows[pos_detection_indices]

        for i in np.arange(num_calib_steps-1):
            detection_windows = _get_network_inputs(img, cur_scale, coords).astype(np.float)
            predictions = classifier.predict([classifier_normalizer.preprocess(detection_windows)])[:,1]
            
            calib_predictions = calibrator.predict([calib_normalizer.preprocess(detection_windows)])
            coords = _calibrate_coordinates(coords, calib_predictions)

            coords, picked = nms(coords, predictions, self._last_stage_iou_thresh)
            centers = centers[picked]

        return coords.astype(np.int32, copy=False), centers

def _get_roomba_proposals(img):
    proposals = []
    centers = []

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ret, red_hue_threshold = cv2.threshold(hsv_img[:, :, 0], RED_ROOMBA_HUE_MIN, RED_ROOMBA_HUE_MAX, cv2.THRESH_BINARY)
    ret, red_saturation_threshold = cv2.threshold(hsv_img[:, :, 1], RED_ROOMBA_SATURATION_MIN, RED_ROOMBA_SATURATION_MAX, cv2.THRESH_BINARY)
    ret, green_hue_threshold = cv2.threshold(hsv_img[:, :, 0], GREEN_ROOMBA_HUE_MIN, GREEN_ROOMBA_HUE_MAX, cv2.THRESH_BINARY)
    ret, green_saturation_threshold = cv2.threshold(hsv_img[:, :, 1], GREEN_ROOMBA_SATURATION_MIN, GREEN_ROOMBA_SATURATION_MAX, cv2.THRESH_BINARY)
    binary = np.bitwise_or(np.bitwise_and(red_saturation_threshold, red_hue_threshold), np.bitwise_and(green_saturation_threshold, green_hue_threshold))

    modified_img, contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= MIN_ROOMBA_AREA:
            moments = cv2.moments(contour)
            centers.append((int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])))
            x, y, w, h = cv2.boundingRect(contour)
            dimensions = np.array([w, h])
            top_left = np.array([x, y]) - MIN_ROOMBA_AREA
            bottom_right = np.array([x, y]) + dimensions + MIN_ROOMBA_AREA
            x_min, y_min = top_left.astype(int)
            x_max, y_max = bottom_right.astype(int)
            proposals.append((x_min, y_min, x_max, y_max))

    return proposals, centers

# Parameters used to simplify access to built-in object detectors.
_ObjectTypes = namedtuple('ObjectTypes', ['FACE', 'ROOMBA'])
ObjectTypes = _ObjectTypes(FACE={BASE_FOLDER_FIELD_NAME: FACE_BASE_FOLDER}, 
                           ROOMBA={BASE_FOLDER_FIELD_NAME: ROOMA_BASE_FOLDER, FAST_DETECT_PARAMS_FIELD_NAME:[_get_roomba_proposals]})

def detect_object(img, object_type, **kwargs):
    """
    Detects objects at multiple scales
    
    Parameters
    ----------
    img : numpy.ndarray
        Image to detect objects in
    object_type : dict
        Member of `detect.ObjectTypes`
    **kwargs : dict, optional
        Optional keyword arguments specifying the cascade's configuration. See `detect.CNNCascadeObjectDetector`
        for more details.
    
    Returns
    -------
    out: numpy.ndarray or a sequence of numpy.ndarray objects
    Returns the coordinates for each detected object's bounding box. Each coordinate set is of the form (x_min, y_min, x_max, y_max).

    If fast detect is supported for `object_type`, then a numpy.ndarray containing the estimated centroid for each object
    might also be available. 
    """

    fast_detect_params = None
    params = object_type

    if FAST_DETECT_PARAMS_FIELD_NAME in object_type:
        fast_detect_params = object_type[FAST_DETECT_PARAMS_FIELD_NAME]
        params = object_type.copy()
        del params[FAST_DETECT_PARAMS_FIELD_NAME]

    cnn_cascade_detector = CNNCascadeObjectDetector(params, **kwargs)
    ret = None

    if fast_detect_params is None:
        ret = cnn_cascade_detector.detect_multiscale(img)
    else:
        ret = cnn_cascade_detector.fast_detect(img, *fast_detect_params)

    return ret


def draw_bounding_boxes(img, boxes, color=DEFAULT_BOUNDING_BOX_COLOR, thickness=DEFAULT_BOUNDING_BOX_THICKNESS):
    """
    Draws bounding boxes on an image.
    
    Parameters
    ----------
    boxes : numpy.ndarray
        Array of bounding boxes. Each row should have the format (x_min, y_min, x_max, y_max)
    color: tuple, optional
        BGR color to draw the bounding boxes with. Default color is bright green.
    thickness: int, optional
        Thickness in pixels of each drawn bounding box's border, defaults to 3.
    
    Returns
    -------
    None
    """

    for (x_min, y_min, x_max, y_max) in boxes:
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color, thickness)