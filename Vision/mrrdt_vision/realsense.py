############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Wrapper around pyrealsense that makes accessing the Intel RealSense R200 camera easier.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

from collections import namedtuple
from timeit import default_timer as timer

import pyrealsense as pyrs
import pyrealsense.constants
import cv2

# all available color resolutions for the R200
AVAILABLE_COLOR_RESOLUTIONS_FOR_R200 = ((320, 240), (640, 480), (1280, 720), (1920, 1080))
# all available depth resolutions for the R200
AVAILABLE_DEPTH_RESOLUTIONS_FOR_R200 = ((320, 240), (480, 360), (628, 468))

# default stream FPS
DEFAULT_FPS = 60
# default stream resolution
DEFAULT_RES = (320, 240)
# RealSense color format prefix
RS_FORMAT_FLAG_PREFIX = 'RS_FORMAT_'

# class for representing different types of RealSense streams
StreamFlags = namedtuple('flags', ['COLOR_STREAM'])
# dictionary of RealSense color format constants.
_FORMATS = {k.replace(RS_FORMAT_FLAG_PREFIX, ''): v for k, v in pyrealsense.constants.rs_format.__dict__.items() if k.startswith(RS_FORMAT_FLAG_PREFIX)}
# dictionary that maps RealSense color formats to OpenCV color conversion flags
_COLOR_CONVERSIONS = {_FORMATS['BGR8']: None,
                      _FORMATS['BGRA8']: cv2.COLOR_BGRA2BGR, 
                      _FORMATS['RGB8']: cv2.COLOR_RGB2BGR, 
                      _FORMATS['RGBA8']: cv2.COLOR_RGBA2BGR,
                      _FORMATS['YUYV']: cv2.COLOR_YUV2BGR_YUYV}

# instance of StreamFlags class that contains all stream types that are currently supported by this wrapper
STREAM_FLAGS = StreamFlags(COLOR_STREAM = 1)
# dictionary that maps RealSense stream types to stream names.
_STREAM_NAMES = {STREAM_FLAGS.COLOR_STREAM: 'color'}

class Service():
    """
    Abstraction of pyrealsense's Service object. Must be used in a context manager.
    """

    def __enter__(self):
        """
        Get a `pyrealsense.Service` instance when this object is instantiated by a context manager.
        """

        self.service = pyrs.Service()
        return self

    def get_service(self):
        """
        Get a reference to a pyrealsense Service object.

        Returns
        -------
        out: pyrealsense.Service
        Returns a reference to a pyrealsense Service object.
        """

        return self.service

    def __exit__(self, *args):
        """
        Stop the service whenever the context manager ends or encounters an exception.
        """

        self.service.stop()

class Device():
    """
    Abstraction of pyrealsense's Device object. Must be used in a context manager.

    Parameters
    ----------
    params: dict
        Keyword arguments to pass to a `pyrealsense.Device` object.
    service: pyrealsense.Service
        Reference to an active pyrealsense service
    """

    def __init__(self, params, service):
        """
        Save device parameters and a reference to a `pyrealsense.Service` object.
        """

        self.params = params
        self.service = service
        self.device = None

    def __enter__(self):
        """
        Get a `pyrealsense.Device` instance when this object is instantiated by a context manager.
        """

        self.device = self.service.Device(**self.params)
        return self.device

    def __exit__(self, *args):
        """
        Stop the device whenever the context manager ends or encounters an exception.
        """

        self.device.stop()

class Streamer():
    """
    Abstraction of an Intel RealSense camera. Simplifies the device to just an object that provides a continuous stream of frames.

    Must be used in a context manager.

    Parameters
    ----------
    flag: int, optional
        One of the flags available in `RealSense.STREAM_FLAGS`, defaults to `STREAM_FLAGS.COLOR_STREAM`.
    res: tuple, optional
        Resolution to stream frames in, defaults to `DEFAULT_RES`. Must be of the form (<width>, <height>).
    fps: int, optional
        Stream frame rate, defaults to `DEFAULT_FPS`
    """

    def __init__(self, flag=STREAM_FLAGS.COLOR_STREAM, res=DEFAULT_RES, fps=DEFAULT_FPS):
        """
        Save parameters and initialize internal attributes
        """

        self.flag = flag
        self.res = res
        self.fps = fps
        self.stream_params = {'width': self.res[0], 'height': self.res[1], 'fps': self.fps}
        self.possible_streams = {STREAM_FLAGS.COLOR_STREAM: pyrs.stream.ColorStream}
        self.service = None
        self.stream = None
        self.device = None
        self.cam = None

    def __enter__(self):
        """
        Start streaming frames from the camera when this object is instantiated by a context manager.
        """

        self.service = Service()
        self.service.__enter__()
        self.stream = self.possible_streams.get(self.flag)(**self.stream_params)
        self.device = Device({'streams': [self.stream]}, self.service.get_service())
        self.cam = self.device.__enter__()
        return self

    def next(self, block=True):
        """
        Get the next available frame(s) from the camera.

        Parameters
        ----------
        block: bool, optional
            If True, halt execution until frame(s) is/are available, otherwise
            return None if there are no frames available.

        Returns
        -------
        out: numpy.ndarray
        Returns frame(s) from the camera or None if `block` is False and there was/were no available frame(s).
        """

        if block:
            self.cam.wait_for_frames()
        else:
            if not self.cam.poll_for_frame():
                return None

        color_conversion_code = _COLOR_CONVERSIONS.get(self.stream.format)
        img = getattr(self.cam, _STREAM_NAMES.get(self.flag))

        if color_conversion_code:
            img = cv2.cvtColor(img, color_conversion_code)

        return img


    def __exit__(self, *args):
        """
        Stop streaming frames from the camera whenever the context manager ends or encounters an exception.
        """

        self.device.__exit__()
        self.service.__exit__()

class LiveDisplay():
    """
    Utility that allows for easy display and modification of live streams.

    Parameters
    ----------
    stream: RealSense.Streamer
        Reference to an active `RealSense.Streamer` object.
    window: visualize.cv2Window
        Reference to a named OpenCV window object for displaying the stream.
    show_fps: bool, optional
        Controls whether or not to show the frame rate of stream live in the title of the window, default is True.

    Notes
    ----------
    LiveDisplay.run() will update its window in an infinite loop until the quit key is hit, the default key for quit is 'q'.

    Examples
    --------
    >>> from visualize import cv2Window
    >>> from RealSense import Streamer, LiveDisplay
    >>> with cv2Window('foo') as win, Streamer() as stream:
    ...     live_stream = LiveDisplay(stream, win)
    ...     live_stream.run()

    The above example will display a color stream from the Intel RealSense in a window titled 'foo'. You should
    also see the frames per second of the stream in parenthesis after 'foo' in the window's title bar. By default,
    pressing the q key on your keyboard will terminate the stream.
    """

    def __init__(self, stream, window, show_fps=True):
        """
        Save parameters and initialize internal attributes
        """

        self.stream = stream
        self.window = window
        self.show_fps = show_fps
        self.original_window_title = window.getTitle()

    def _update_fps(self, fps):
        """
        Sets the stream's frame rate in the title bar.

        Parameters
        ----------
        fps: int
            The stream's frame rate in frames per second.

        Returns
        -------
        None
        """

        self.window.setTitle('%s (%d FPS)' % (self.original_window_title, fps))

    def get_next_frame(self):
        """
        Get the next available frame from the camera.

        Returns
        -------
        out: numpy.ndarray
        Returns a frame retrieved from the camera.
        """

        return self.stream.next()

    def run(self, callback=None, key_to_quit='q'):
        """
        Starts the infinite loop that updates and handles events for the GUI that displays the live stream.

        Parameters
        ----------
        callback: callable, optional
            Function called whenever a new frame is retrieved, default behavior is to not have any function be called.
            When present, `callback` will receive the new frame as an argument.
        key_to_quit: str, optional
            Key which will terminate the loop and associated window when pressed, default key for this action is 'q'.
        
        Returns
        -------
        None
        """

        should_quit = False
        start = timer()
        frames = 0

        while not should_quit:
            img = self.get_next_frame()

            if callback is not None: 
                callback(img)

            self.window.show(img)

            frames += 1

            if self.window.getKey() == key_to_quit: 
                should_quit = True

            if (timer() - start >= 1):
                self._update_fps(frames)
                start = timer()
                frames = 0