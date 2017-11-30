import os
import cv2
import numpy as np
import tensorflow as tf

import display

from sklearn.preprocessing import normalize
from timeit import default_timer as timer

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# default bounding box color (green)
DEFAULT_BOUNDING_BOX_COLOR = (0, 255, 0)
# default bounding box thickness
DEFAULT_BOUNDING_BOX_THICKNESS = 3
# GPU-accelerated module path
GPU_MODULE_PATH = 'threshold_gpu.so'
# whether or not to enable GPU-accelerated thresholding 
GPU_ACCELERATED = os.path.isfile(GPU_MODULE_PATH)
# parameter that determines the precision of the polygon approximation.
EPSILON = .03
# length of directional arrow
ARROW_LENGTH = 50
# color of directional arrow
ARROW_COLOR = (255, 0, 255)
# directional arrow thickness
ARROW_THICKNESS = 3
# key to quit
QUIT_KEY = 'q'
# image(s) for this file's unit test
TEST_VIDEO_PATH = '../data/IARC.mp4'

CWD_PATH = os.getcwd()
DATA_DIR = 'ssd'
PATH_TO_CKPT = os.path.join(CWD_PATH, DATA_DIR, 'ssd_300_roomba_v1.pb')
PATH_TO_LABELS = os.path.join(CWD_PATH, DATA_DIR, 'object-detection.pbtext')
NUM_CLASSES = 1

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES)
category_index = label_map_util.create_category_index(categories)

def detect_objects(image_np, sess, detection_graph):
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    scores = detection_graph.get_tensor_by_name('detection_scores:0')
    classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    (boxes, scores, classes, num_detections) = sess.run(
        [boxes, scores, classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})

    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        category_index,
        use_normalized_coordinates=True,
        line_thickness=8)
    
    return image_np

class Roomba():
    def __init__(self, bounding_box, center, orientation):
        self._bounding_box = bounding_box
        self._center = center
        self._orientation = orientation
        self._bounding_box_color = DEFAULT_BOUNDING_BOX_COLOR
        self._bounding_box_thickness = DEFAULT_BOUNDING_BOX_THICKNESS
    
    @property
    def bounding_box(self):
        return self._bounding_box

    @property
    def center(self):
        return self._center

    @property
    def orientation(self):
        return self._orientation

    def draw(self, img, show_orientation=True):
        x_min, y_min, x_max, y_max = self.bounding_box
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), self._bounding_box_color, self._bounding_box_thickness)

        if show_orientation:
            cv2.arrowedLine(img, tuple(self.center.astype(int)), tuple((self.center+ARROW_LENGTH*self.orientation).astype(int)), ARROW_COLOR, ARROW_THICKNESS)

class RoombaDetector():
    def __init__(self):
        self._detection_graph = tf.Graph()

        with self._detection_graph.as_default():
            graph_def = tf.GraphDef()

            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

            self._sess = tf.Session(graph=self._detection_graph)

    @property
    def session(self):
        return self._sess

    @property
    def detection_graph(self):
        return self._detection_graph

    def detect(self, img):
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_np_expanded = np.expand_dims(rgb_img, axis=0)
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        (boxes, scores, classes, num_detections) = self.session.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        vis_util.visualize_boxes_and_labels_on_image_array(
            img,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8)

if __name__ == '__main__':
    # unit test
    detector = RoombaDetector()
    frames = 0
    start = timer()

    with display.VideoReader(TEST_VIDEO_PATH) as video, display.Window(TEST_VIDEO_PATH) as window:
        try:
            should_quit = False

            for frame in video:
                if should_quit:
                    break
                if frame is None:
                    break
                
                detector.detect(frame)
                window.show(frame)
                should_quit = window.is_key_down(QUIT_KEY)

                frames += 1

                if (timer()-start >= 1):
                    window.set_title('%s (%d FPS)' % (TEST_VIDEO_PATH, frames))
                    start = timer()
                    frames = 0
        except KeyboardInterrupt as e:
            pass