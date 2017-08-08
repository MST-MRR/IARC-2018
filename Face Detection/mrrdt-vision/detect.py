############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science Technology
# Summer 2017
# Christopher O'Toole

"""
Contains the functions neccessary to detect objects at multiple scales

Some mathematical formulas and underlying methods were inspired or taken from [1].

References
----------
.. [1] Li, H., Lin, Z., Shen, X., Brandt, J., & Hua, G. (2015). A convolutional neural network cascade for face detection. 
       In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (pp. 5325-5334).
"""

from __future__ import absolute_import, division, print_function, unicode_literals

import os

import cv2
import numpy as np

from .model import MODELS

from .data import DatasetManager, num_detection_windows_along_axis, squash_coords, MIN_OBJECT_SCALE, OFFSET, SCALES, CALIB_PATTERNS_ARR, OBJECT_DATABASE_PATHS, NEGATIVE_DATABASE_PATHS

# default IoU threshold for non-maximum suppression
IOU_THRESH = .5
# iou threshold to use for the 3rd stage of the cascade
LAST_STAGE_IOU_THRESH = .2
# minimum score for 12-net to classify a region as positive
NET_12_THRESH = .009
# minimum score for 24-net to classify a region as positive
NET_24_THRESH = .2
# minimum score for 48-net to classify a region as positive
NET_48_THRESH = .5

# default number of pyramid levels to use for detect_multiscale
NUM_PYRAMID_LEVELS = 5
# amount to shrink the previous pyramid level's image by
PYRAMID_DOWNSCALE = 1.25

def _iou(boxes, box, area=None):
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

def _nms(boxes, predictions, iou_thresh=IOU_THRESH):
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

        int_over_union = _iou(boxes[idxs[:-1]], pick)
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

    suppressed_boxes = np.zeros((0, *boxes.shape[1:]))
    prev_idx = 0
    new_pyr_idxs = []
    picked = []

    for cur_idx in pyr_idxs:
        local_suppressed_boxes, local_picked = _nms(boxes[prev_idx:cur_idx], predictions[prev_idx:cur_idx], iou_thresh)
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

# cached calibrator and classifier objects for each stage of the cascade
MODELS = [(MODELS[False][stage_idx], MODELS[True][stage_idx]) for stage_idx in np.arange(len(SCALES))]
# cached input normalizers for each stage's classifier and calibrator
NORMALIZERS = []
# cached threshold values for each stage
THRESHOLDS = (NET_12_THRESH, NET_24_THRESH, NET_48_THRESH)

def detect_multiscale(img, max_stage_idx=len(SCALES)-1, min_object_scale=MIN_OBJECT_SCALE, **dataset_manager_params):
    """
    Detect objects at multiple scales
    
    Parameters
    ----------
    img : numpy.ndarray
           Image to detect objects in
    max_stage_idx: int, optional
           Cascade stage to stop at. Default behavior is to use the entire cascade.
    min_object_scale : int, optional
           `min_object_scale`x`min_object_scale` is the smallest region for which the detector will be effective. However,
           good results may still be acheived with slightly smaller regions due to bounding box calibration. Default is `MIN_OBJECT_SCALE`.
    dataset_manager_params: dict, optional
            Optionally specify a dataset to use in place of default one.

    Returns
    -------
    out: numpy.ndarray
    Returns the coordinates for each detected object's bounding box. Each coordinate set is of the form (x_min, y_min, x_max, y_max) 
    """

    classifier_inputs = []
    calibrator_inputs = []

    for stage_idx in np.arange(0, max_stage_idx + 1):
        cur_scale = SCALES[stage_idx][0]
        classifier, calibrator = MODELS[stage_idx]

        if os.path.isfile(OBJECT_DATABASE_PATHS[stage_idx]) and os.path.isfile(NEGATIVE_DATABASE_PATHS[stage_idx]) and len(NORMALIZERS) == stage_idx:
            NORMALIZERS.append(tuple((DatasetManager(MODELS[stage_idx][is_calib], **dataset_manager_params).get_normalizer() for is_calib in (0, 1))))

        if stage_idx == 0:
            detection_window_generator = _get_detection_windows(img, cur_scale)
            total_num_detection_windows = next(detection_window_generator)
            detection_windows = np.zeros((total_num_detection_windows, cur_scale, cur_scale, 3))
            coords = np.zeros((total_num_detection_windows, 4))
            pyr_idxs = []
            prev_pyr_level = 0

            for i, (pyr_level, x_min, y_min, x_max, y_max, detection_window) in enumerate(detection_window_generator):
                detection_windows[i] = detection_window
                coords[i] = (x_min, y_min, x_max, y_max)
                coords[i] *= PYRAMID_DOWNSCALE ** pyr_level

                if pyr_level != prev_pyr_level:
                    prev_pyr_level = pyr_level
                    pyr_idxs.append(i)

            pyr_idxs.append(len(detection_windows))
            coords *= min_object_scale/cur_scale
        else:
            for i in np.arange(0, stage_idx):
                classifier_inputs[i] = NORMALIZERS[i][0].preprocess(_get_network_inputs(img, classifier_inputs[i].shape[1], coords).astype(np.float))

        classifier_normalizer, calib_normalizer = NORMALIZERS[stage_idx]

        detection_windows = detection_windows if stage_idx == 0 else _get_network_inputs(img, cur_scale, coords).astype(np.float)
        classifier_inputs.insert(0 if stage_idx < 2 else stage_idx, classifier_normalizer.preprocess(detection_windows))
        predictions = classifier.predict(classifier_inputs)[:,1]
        pos_detection_indices = np.where(predictions>=THRESHOLDS[stage_idx])

        calibrator_inputs = [calib_normalizer.preprocess(detection_windows[pos_detection_indices])]
        calib_predictions = calibrator.predict(calibrator_inputs)
        coords = _calibrate_coordinates(coords[pos_detection_indices], calib_predictions)

        if stage_idx == len(SCALES)-1:
            coords, picked = _nms(coords, predictions[pos_detection_indices], iou_thresh=LAST_STAGE_IOU_THRESH)
        else:
            coords, picked, pyr_idxs = _local_nms(coords, predictions[pos_detection_indices], pyr_idxs)

        if stage_idx == max_stage_idx:
            return coords.astype(np.int32, copy=False)
