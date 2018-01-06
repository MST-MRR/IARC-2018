#!/usr/bin/env python3.5
############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Fall 2017
# Christopher O'Toole

import cv2
import numpy as np
import os
import argparse

import xml.etree.ElementTree as ET
from lxml import etree

DEFAULT_WINDOW_NAME = 'Line Annotator'
DEFAULT_IMAGE_EXTENSION = '.jpg'

LINE_COLOR = (0, 255, 0)
LINE_THICKNESS = 2

ROOT_NAME = 'annotation'
OBJECT_ELEMENT_NAME = 'object'
ANNOTATION_DEFAULT_DIR = 'Annotations/'

QUIT_KEY = 'q'
CLEAR_KEY = 'c'
NEXT_KEY = 'n'
PREV_KEY = 'p'
UNDO_KEY = 'u'

def _generate_xml_element_with_text(name, text):
    element = etree.Element(name)
    element.text = str(text)
    return element

def _generate_object_annotation(line):
    xmin, ymin, xmax, ymax = line
    m = (ymax-ymin)/(xmax-xmin+1e-9)
    theta = np.arctan2(ymax-ymin, xmax-xmin)
    rho = np.abs(ymin+m*xmin)/np.sqrt(m**2+1)

    obj = etree.Element(OBJECT_ELEMENT_NAME)
    
    params = etree.Element('params')
    params.append(_generate_xml_element_with_text('rho', rho))
    params.append(_generate_xml_element_with_text('theta', theta))
    params.append(_generate_xml_element_with_text('xmin', xmin))
    params.append(_generate_xml_element_with_text('ymin', ymin))
    params.append(_generate_xml_element_with_text('xmax', xmax))
    params.append(_generate_xml_element_with_text('ymax', ymax))
    obj.append(params)

    return obj


def _generate_annotation_from_image(img, lines, file_path, overwrite=False):
    height, width, depth = img.shape
    filename = os.path.splitext(os.path.basename(file_path))[0] + '.jpg'

    root = etree.Element(ROOT_NAME)
    root.append(_generate_xml_element_with_text('filename', filename))
    
    size = etree.Element('size')
    size.append(_generate_xml_element_with_text('width', width))
    size.append(_generate_xml_element_with_text('height', height))
    size.append(_generate_xml_element_with_text('depth', depth))
    root.append(size)

    for line in lines:
        root.append(_generate_object_annotation(line))

    if not overwrite:
        assert not os.path.exists(file_path), '_generate_annotation_from_image(): %s already exists!' % (file_path,)

    with open(file_path, 'wb') as out_file:
        out_file.write(etree.tostring(root, pretty_print=True))

def _generate_annotation_from_image_file(img_path, lines, annotation_dir=None, overwrite=False):
    img_file_name, img_extension = os.path.splitext(img_path)
    img = cv2.imread(img_path)
    assert img is not None, '_generate_annotation_from_image_file(): could not read image at %s' % (img_path,)
    
    if annotation_dir is None:
        _generate_annotation_from_image(img, lines, img_file_name + '.xml', overwrite=overwrite)
    else:
        img_folder = os.path.dirname(img_file_name)
        img_name = os.path.basename(img_file_name)
        annotation_folder = os.path.join(img_folder, annotation_dir)

        if not os.path.exists(annotation_folder):
            os.mkdir(annotation_folder)
        
        _generate_annotation_from_image(img, lines, os.path.join(annotation_folder, img_name) + '.xml', overwrite=overwrite)


def _parse_annoation(path):
    tree = ET.parse(path)
    root = tree.getroot()

    lines = []

    for obj in root.findall('object'):
        params = obj.find('params')
        lines.append((int(params.find('xmin').text), int(params.find('ymin').text), int(params.find('xmax').text), int(params.find('ymax').text)))
    
    return lines

def get_ground_truth_from_annotation(path):
    tree = ET.parse(path)
    root = tree.getroot()

    lines = []

    for obj in root.findall('object'):
        params = obj.find('params')
        lines.append((float(params.find('rho').text), float(params.find('theta').text)))
    
    return lines

class LineAnnotator():
    def __init__(self, folder, name=DEFAULT_WINDOW_NAME, window_type=cv2.WINDOW_AUTOSIZE):
        assert os.path.isdir(folder), 'LineAnnotator(): %s is not a directory' % (folder,)
        self._name = name
        self._title = ''
        self._type = window_type
        self._folder = folder
        self._img_paths = [os.path.join(folder, filename) for filename in os.listdir(folder) if DEFAULT_IMAGE_EXTENSION in filename]
        assert self._img_paths, 'LineAnnotator(): no JPEG images were found in directory %s' % (folder,)
        self._cur_img = None
        self._prev_img = None
        self._cur_lines = []
        self._changed = False
        self._line_in_progress = []
        self._annotations = {}
        self._load_saved_annotations()
        self.index = 0
    
    def _load_saved_annotations(self):
        if os.path.isdir(self.annotation_dir):
            for filename in os.listdir(self.annotation_dir):
                key = os.path.join(self._folder, os.path.basename(os.path.splitext(filename)[0]) + DEFAULT_IMAGE_EXTENSION)
                self._annotations[key] = _parse_annoation(os.path.join(self.annotation_dir, filename))


    def _next(self):
        self.index = self.index + 1
    
    def _prev(self):
        self.index = self.index - 1

    def _remove_annotation_file_when_empty(self):
        if hasattr(self, '_index'):
            annotation_file_path = os.path.join(self.annotation_dir, os.path.basename(os.path.splitext(self.cur_img_path)[0]) + '.xml')
            if not self._cur_lines and os.path.isfile(annotation_file_path):
                os.remove(annotation_file_path)

    def _undo(self):
        if not self._line_in_progress and self._cur_lines:
            self._changed = True
            del self._cur_lines[-1]
        

    def _clear(self):
        self._cur_img = None
        self._changed = True
        
        if self.cur_img_path in self._annotations:
            del self._annotations[self.cur_img_path]
        
        self.index = self.index

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._line_in_progress = [x, y]*2
        elif event == cv2.EVENT_MOUSEMOVE and self._line_in_progress:
            self._line_in_progress[2] = x
            self._line_in_progress[3] = y
        elif event == cv2.EVENT_LBUTTONUP and self._line_in_progress:
            self._changed = True
            self._cur_lines.append(self._line_in_progress)
            self._line_in_progress = []

    def _get_key(self):
        return chr(cv2.waitKey(1) & 0xFF)

    def _draw(self, img, coords, color=LINE_COLOR, thickness=LINE_THICKNESS):
        xmin, ymin, xmax, ymax = coords
        cv2.line(img, (xmin, ymin), (xmax, ymax), color, thickness)

    @property
    def type(self):
        return self._type

    @property
    def name(self):
        return self._name

    @property
    def index(self):
        return self._index

    @property
    def title(self):
        return self._title

    @title.setter
    def title(self, value):
        self._title = str(value)
        cv2.setWindowTitle(self.name, '%s%s' % (self.name, self.title))

    @property
    def annotation_dir(self):
        return os.path.join(self._folder, ANNOTATION_DEFAULT_DIR)

    @index.setter
    def index(self, value):
        if self._cur_img is not None and self._cur_lines and self._changed:
            _generate_annotation_from_image_file(self.cur_img_path, self._cur_lines, self.annotation_dir, True)

        self._remove_annotation_file_when_empty()

        self._index = value % len(self._img_paths)
        self.title = ' (%d/%d)' % (self.index+1, len(self._img_paths))
        self._cur_img = cv2.imread(self.cur_img_path)

        if not self.cur_img_path in self._annotations:
            self._annotations[self.cur_img_path] = []
        
        self._cur_lines = self._annotations[self.cur_img_path]
        assert self._cur_img is not None, 'LineAnnotator(): could not read image at %s' % (self.cur_img_path,)
        self._changed = False

    @property
    def cur_img(self):
        return self._cur_img

    @property
    def cur_img_path(self):
        return self._img_paths[self._index]

    def update(self):
        should_quit = False

        if self._changed or self._line_in_progress or self._prev_img is not self._cur_img:
            self._prev_image = self._cur_img
            cur_img_copy = self._cur_img.copy()
            
            for line in self._cur_lines + [self._line_in_progress]:
                if line:
                    self._draw(cur_img_copy, line)
            
            cv2.imshow(self.name, cur_img_copy)
        
        key = self._get_key()

        if key == QUIT_KEY:
            should_quit = True
        elif key == NEXT_KEY:
            self._next()
        elif key == PREV_KEY:
            self._prev()
        elif key == CLEAR_KEY:
            self._clear()
        elif key == UNDO_KEY:
            self._undo()

        return should_quit

    def __enter__(self):
        cv2.namedWindow(self.name, self.type)
        cv2.setMouseCallback(self.name, self._mouse_callback)
        return self

    def __exit__(self, *args):
        if self._changed:
            self.index = self.index + 1
        
        cv2.destroyWindow(self.name)

    
def run(folder, **kwargs):
    with LineAnnotator(folder, **kwargs) as annotator:
        while not annotator.update():
            pass

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-f', '--folder', required=True, help='path to folder in which the images to be annotated are stored')
    args = vars(arg_parser.parse_args())
    run(args['folder'])