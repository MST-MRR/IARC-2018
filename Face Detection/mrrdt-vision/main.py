############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Main module for this project. 

Usage
----------
First ensure that you are in the same directory that the parent folder for this project is stored in. Then you may execute the module
via a command analagous to the following one:

username@username-pc-name:~$ python -m trainer.main --help

The example above will display usage information for every currently available command-line option. --help may be
replaced with any valid sequence of command-line options.

Notes
----------
You should be able to train models with Python 2.7.13, but it is recommended that you always run this module 
with the Python 3.5.2 interpreter since Python 2.7.13 support may not be available when using features outside
of training.

In theory, this module should run on any system since no Ubuntu-specific modules are used, but keep in mind
that this module was developed almost exclusively on Ubuntu 16.04 LTS and no attempts have been made to test it
extensively on other platforms. Also, the dependencies of this module may have varying levels of support and 
availability on other operating systems, so it is possible that this module will either work at a lower quality or 
not run at all on certain platforms.

OpenCV must be built with graphical support enabled for the command-line options --visualize and --live to work properly.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from optparse import OptionParser
from timeit import default_timer as timer

# window title used in -v/--visualize mode
WINDOW_TITLE = 'Object Detector Test'
# window title used in -l/--live mode
LIVE_WINDOW_TITLE = 'RealSense Test'

# display how many seconds it took to get predictions for a single frame
PROFILE = True
# set to true when testing experimental code.
DEBUG = False

# bounding box color
GREEN = (0, 255, 0)
# bounding box thickness
THICKNESS = 3

if __name__ == '__main__':
    from .data import DatasetManager
    from .model import MODELS

    # parse command-line arguments
    parser = OptionParser()
    parser.add_option('-s', '--stage', dest='stageIdx', help='Cascade stage index', metavar = '[0, 2]', default = 2)
    parser.add_option('-c', '--calib', action='store_true', dest = 'isCalib', help='Perform actions with the calibrator instead of the classifier.', default = False)
    parser.add_option('-v', '--visualize',  action='store_true', dest = 'visualizeMode', help='View output of detector on different test images', default = False)
    parser.add_option('-l', '--live',  action='store_true', dest = 'liveMode', help='Test detector on a live Intel RealSense Stream', default = False)
    parser.add_option('-e', '--eval', action='store_true', dest='evalMode', help="Summarize the performance of the specfiied model using plots and applicable metrics.", default = False)
    parser.add_option('-t', '--train', action='store_true', dest='trainMode', help='Train either the classifier or calibrator for the given stage', default = False)

    (options, args) = parser.parse_args()

    # get model for training or evaluation when one of those options are specified
    if options.trainMode or options.evalMode:
        model = MODELS[options.isCalib][int(options.stageIdx)]
        dataset_manager = DatasetManager(model)

    def prediction_callback(img):
        """
        Make and display predictions on an image

        Parameters
        ----------
        img: numpy.ndarray
            Image to make and display predictions on. Note that the image will have bounding boxes drawn on it in the
            calling function after this function returns.

        Notes
        ----------
        If `main.PROFILE` is True, this function will display how many seconds it took to make predictions on `img`
        
        Returns
        -------
        None
        """

        import cv2
        from .detect import detect_multiscale
        start = timer()
        detections = detect_multiscale(img, int(options.stageIdx), base_folder='face')

        if PROFILE:
            print('Prediction took %fs' % (timer() - start,))
        
        for (x_min, y_min, x_max, y_max) in detections: 
            cv2.rectangle(img, (x_min, y_min), (x_max, y_max), GREEN, THICKNESS)

    if options.visualizeMode:
        from .visualize import visualizer
        from .data import get_test_image_paths
        visualizer(get_test_image_paths(), prediction_callback, WINDOW_TITLE)
    elif options.trainMode:
        from .train import train
        train(model, dataset_manager)
    elif options.liveMode:
        from visualize import cv2Window
        from RealSense import Streamer, LiveDisplay

        with cv2Window(LIVE_WINDOW_TITLE) as win, Streamer() as stream:
            live_stream = LiveDisplay(stream, win)
            live_stream.run(prediction_callback)
    elif options.evalMode:
        from .eval import ModelEvaluator
        
        with ModelEvaluator(model, dataset_manager) as evaluator:
            evaluator.summary()
