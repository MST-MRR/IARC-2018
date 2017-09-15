############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

import os
import pickle
import sys
sys.path.append('../..')

import mrrdt_vision
import cv2
import numpy as np

# folder containing images of roombas.
POSITIVE_IMAGE_FOLDER = '/home/christopher/IARC-2018/examples/roomba_detection/Positives'
# title of the window which displays the detection results.
WINDOW_TITLE = 'Roomba Detection Experimental'

def get_roomba_image_paths(pos_img_folder=POSITIVE_IMAGE_FOLDER):
    """
    Retrieves the path of every image file in the directory described by `pos_img_folder`.

    Parameters
    ----------
    pos_img_folder: str, optional
        Path to the folder which containing the targeted images, defaults to `POSITIVE_IMAGE_FOLDER`.

    Returns
    -------
    out: list
    Returns a list of paths that correspond to every file in `pos_img_folder`.
    """

    return [os.path.join(pos_img_folder, file_name) for file_name in os.listdir(pos_img_folder)]


class RegionSelector():
    DEFAULT_SELECTION_BOX_COLOR = (0, 255, 0)
    DEFAULT_SELECTION_BOX_THICKNESS = 3

    def __init__(self, title=WINDOW_TITLE, img_paths=get_roomba_image_paths(), selection_color=DEFAULT_SELECTION_BOX_COLOR,
                 selection_box_thickness=DEFAULT_SELECTION_BOX_THICKNESS):
        self.title = title
        self.idx = 0
        self.img_paths = img_paths
        self.selection_color = selection_color
        self.selection_box_thickness = selection_box_thickness
        self.quit = False
        self.mouse_button_down = False

        self.set_cur_img()

    def __enter__(self):
        self.window = mrrdt_vision.Window(self.title).__enter__()
        self.window.set_mouse_callback(self.set_selection)
        return self

    def __exit__(self, *args):
        self.window.__exit__()

    def set_selection(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.region = [(x, y), (x, y)]
            self.mouse_button_down = True
        elif self.mouse_button_down and (event == cv2.EVENT_MOUSEMOVE or event == cv2.EVENT_LBUTTONUP):
            self.region[1] = (x, y)
            if event == cv2.EVENT_LBUTTONUP:
                self.mouse_button_down = False

    def set_cur_img(self):
        self.cur_img = cv2.imread(self.img_paths[self.idx])
        self.region = [(0, 0), (0, 0)]
    
    def update(self):
        key_pressed = self.window.get_key()
        cur_img_copy = self.cur_img.copy()
        x_min, y_min = self.region[0]
        x_max, y_max = self.region[1]

        if (x_min - x_max) and (y_min - y_max):
            cv2.rectangle(cur_img_copy, self.region[0], self.region[1], self.selection_color, self.selection_box_thickness)

            text_pos = (x_min + (x_max - x_min)//2, y_min + (y_max - y_min)//2)
            x, y, w, h = mrrdt_vision.squash_coords(self.cur_img, x_min, y_min, x_max - x_min, y_max - y_min)
            hsv_img = cv2.cvtColor(cur_img_copy, cv2.COLOR_BGR2HSV)[y:y+h, x:x+w]
            if w > 0 and h > 0:
                hue, saturation = (hsv_img[:, :, 0], hsv_img[:, :, 1])
                text = 'Hue [%d, %d], Saturation [%d, %d]' % (np.amin(hue), np.amax(hue), np.amin(saturation), np.amax(saturation))
                cv2.putText(cur_img_copy, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 2, cv2.LINE_AA)

        if key_pressed == 'n':
            self.idx += 1
            self.idx = self.idx % len(self.img_paths)
            self.set_cur_img()
        elif key_pressed == 'p':
            self.idx -= 1
            if self.idx < 0:
                self.idx = len(self.img_paths)
            self.set_cur_img()
        elif key_pressed == 'q':
            self.quit = True

        self.window.show(cur_img_copy)
        return not self.quit


def main():
    with RegionSelector() as region_selector:
        while region_selector.update():
            pass

if __name__ == '__main__':
    main()