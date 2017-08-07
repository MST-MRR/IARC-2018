############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

"""
A collection of generic utility functions to make Python development easier.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import inspect 
import h5py
import tempfile
import os
import sys
import random

# random number range used for random filename generation
RAND_LOW, RAND_HIGH = (0, sys.maxsize)

def static_vars(**kwargs):
    """
    Python decorator for adding static variables to functions.

    Parameters
    ----------
    **kwargs: dict
        Static variables to add as a dicionary of name-value pairs.
    
    Returns
    -------
    out: function
    Returns the decorated function with the static variables set and initialized as specified by `**kwargs`.
    """
    
    def decorate(func):
        """
        Handles adding the specified static variables to the decorated function.
        """
        
        for k, v in kwargs.items():
            setattr(func, k, v)
            
        return func
    
    return decorate

def get_default_params(func):
    """
    Gets names and values of `func`'s default parameters.

    Parameters
    ----------
    **kwargs: dict
        Static variables to add as a dicionary of name-value pairs.
    
    Returns
    -------
    out: function
    Returns the decorated function with the static variables set and initialized as specified by `**kwargs`.
    """
    
    args, varargs, keywords, defaults = inspect.getargspec(func)
    return dict(zip(args[len(args)-len(defaults):], defaults))

class TempH5pyFile(h5py.File):
    """
    Facilitates creation of temporary h5py files

    Parameters
    ----------
    *args: tuple
	  Positional arguments to pass to the `h5py.File` constructor.
    **kwargs: dict
      Keyword arguments to pass to the `h5py.File` constructor.
    
    Notes
    ----------
    This class must be instantiated with a context manager.
    """

    def __init__(self, *args, **kwargs):
        """
        Saves the passed file creation arguments and finds a random unused filename.
        """

        self.args = args
        self.kwargs = kwargs
        self.file_path = self._generate_file_path()

        while os.path.isfile(self.file_path):
            self.file_path = self._generate_file_path()

    def _generate_file_path(self):
        """
        Generate a temporary file path at random.

        Returns
        -------
        out: str
        Returns the temporary file path generated.
        """

        return os.path.join(tempfile.gettempdir(), str(random.randint(RAND_LOW, RAND_HIGH)) + '.hdf')

    def get_path(self):
        """
        Get the path to the temporary file represented by this class instance.

        Returns
        -------
        out: str
        Returns the temporary file's path.
        """

        return self.file_path

    def __enter__(self):
        """
        Constructor used when instantiated with a context manager. Sets up attributes and objects used by the superclass.
        """

        super(TempH5pyFile, self).__init__(self.file_path, *self.args, **self.kwargs)
        return self

    def close(self):
        """
        Close the temporary file's handle and remove the temporary file from existence.

        Returns
        -------
        None
        """

        try:
            super(TempH5pyFile, self).close()
        finally: 
            try:
                os.remove(self.file_path)
            except:
                pass

    def __exit__(self, *args):
        """
        Called by the context manager on exception or upon successful execution of its code block. Frees up resources used by this object.
        """

        self.close()