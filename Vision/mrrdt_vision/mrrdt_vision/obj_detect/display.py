############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
Tools for visualizing the results of object detection and computer vision algorithms on a sequence of images.
"""

import errno
import os

import numpy as np
import cv2

# default title for the visualizer function's window
DEFAULT_VISUALIZER_WINDOW_TITLE = 'Visualizer'

class Window():
    """
    Lightweight wrapper for named OpenCV windows.

    Parameters
    ----------
    name: str
        Used as both the title and the name of the created window.
    window_type: int, optional
        One or more OpenCV big flags specifying the appearance of the created window, defaults to `cv2.WINDOW_AUTOSIZE`.

    Notes
    ----------
    To update the window and to prevent the window from becoming unresponsive, you must call either is_key_down or get_key in a 
    loop. Only calling window.show in a loop is not sufficient.
    """

    def __init__(self, name, window_type=cv2.WINDOW_AUTOSIZE):
        """
        Saves arguments for the window's construction and initializes internal attributes.
        """

        self.name = name
        self.title = name
        self.type = window_type
        self.state = {}

    def __enter__(self):
        """
        Creates a new window when this object is instantiated by a context manager.
        """

        cv2.namedWindow(self.name, self.type)
        return self

    def __exit__(self, *args):
        """
        Destroys the window created earlier whenever the context manager ends or encounters an exception.
        """

        cv2.destroyWindow(self.name)

    def get_title(self):
        """
        Get the window's title

        Returns
        -------
        out: str
        Returns the window's title
        """

        return self.title

    def set_title(self, new_title):
        """
        Sets the window's title

        Returns
        -------
        None
        """

        self.title = new_title
        cv2.setWindowTitle(self.name, self.title)

    def is_key_down(self, key):
        """
        Returns whether or not a specific key was pressed while the window was active.

        Parameters
        ----------
        key: str
            Key to test for

        Returns
        -------
        out: bool
        Returns True if `key` was pressed, False otherwise.
        """

        return cv2.waitKey(1) & 0xFF == ord(key)

    def set_mouse_callback(self, func):
        """
        Sets a callback for handling mouse events when the window referred to by the calling object has focus.

        Parameters
        ----------
        func: callable
            Callback which will be called upon the occurrence of any mouse-related event.

        Returns
        -------
        None
        """
        
        cv2.setMouseCallback(self.name, func)

    def create_trackbar(self, name, value, max, on_change=lambda val: None):
        """
        Creates a trackbar for the window referred to by the calling object.

        Parameters
        ----------
        name: str
            Label for the newly created trackbar
        value: int
            The slider's initial position value
        max: int
            The maximum possible value for the slider.
        on_change: callable
            Function called when the slider's position is changed. Defaults to a dummy function
            which does nothing.

        Returns
        -------
        None
        """

        cv2.createTrackbar(name, self.get_title(), value, max, on_change)


    def get_key(self):
        """
        Returns any keypress made while the window was active.
        
        Returns
        -------
        out: str
        Returns the string representation of the pressed key, or ÿ if no key was pressed.
        """

        return chr(cv2.waitKey(1) & 0xFF)

    def show(self, mat):
        """
        Displays the image contained within `mat`.

        Parameters
        ----------
        mat: numpy.ndarray
            Image to display

        Returns
        -------
        None
        """

        cv2.imshow(self.name, mat)

class VideoReader():
    '''
    Utility class for reading a video file with OpenCV

    Parameters
    ----------
    file_path: str
        Path to the video file.

    Raises
    ------
    FileNotFoundError
        If the file referred to by `file_path` could not be found.

    '''

    def __init__(self, file_path):
        '''Intialize internal attributes'''
        self.file_path = file_path

        if not os.path.isfile(file_path):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), file_path)

    def __enter__(self):
        '''Begin streaming from the video on disk once instantiated by a context manager'''
        self.cap = cv2.VideoCapture(self.file_path)
        return self

    def __iter__(self):
        '''VideoReader is its own iterator, so this method simply returns a reference to its calling object.'''
        return self

    def __next__(self):
        '''
        Gets the next frame from the video.

        Returns
        -------
        out: numpy.ndarray
        Returns the next frame.

        Raises
        ------
        StopIteration
            If the end of the video has been reached.
        '''

        if not self.cap.isOpened():
            raise StopIteration

        return self.cap.read()[1]

    # here for compatibility with Python 2
    def next(self):
        '''See `VideoReader.next` for this function's documentation'''
        return self.__next__()

    def __exit__(self, *args):
        '''Stop streaming from the video on disk once the context manager's scope ends or an exception is hit'''
        self.cap.release()

def visualizer(images, callback=None, win_title=DEFAULT_VISUALIZER_WINDOW_TITLE):
    """
    Helper function for traversing and displaying a set of images.

    Parameters
    ----------
    images: Sequence of type str or numpy.ndarray
        Either a set of image paths for the function to load in or a set of preloaded images. 
    callback: callable, optional
        Optional function for modifying or analyzing an image before it is displayed, default behavior is to display the image
        without any analysis or modification applied.
    win_title: str, optional
        Title for the window that the images will be displayed in, default is `visualize.DEFAULT_VISUALIZER_WINDOW_TITLE`

    Notes
    ----------
    To quit the visualizer, press the "q" key on your keyboard while the visualizer's window is active.

    Returns
    -------
    None
    """


    should_quit = False
    length = len(images)
    i = 0
    img = None

    with Window( win_title ) as window:
        while not should_quit:
            if isinstance(images[i], np.ndarray):
                img = images[i]
            elif isinstance(images[i], str):
                img = cv2.imread(images[i])

                if img is None:
                    i = (i + 1) % length
                    continue

            if callback:
                callback(img)

            window.show(img)
            key = window.get_key()

            while key not in 'npq':
                key = window.get_key()

            if key == 'n':
                i = ( i + 1 ) % length
            elif key == 'p':
                i = i - 1 if i > 0 else length-1
            elif key == 'q':
                should_quit = True