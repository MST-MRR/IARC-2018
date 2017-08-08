############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'''
A small library for maintaining data that persists between program runs.
'''

import os
import pickle
import copy

# default global storage file path
GLOBAL_STORAGE_FILE_PATH = 'global_storage'
# pickle storage protocol compatible with Python 2
PYTHON_2_COMPATIBLE_PICKLE_PROTOCOL = 2

class GlobalStorage():
    '''
    Manages data that can be accessed globally within a project.

    Parameters
    ----------
    storage_file_path: str, optional
        Path to the data storage file, defaults to `GLOBAL_STORAGE_FILE_PATH`.
    '''

    def __init__(self, storage_file_path=GLOBAL_STORAGE_FILE_PATH):
        """
        Loads in the saved data storage if found and initializes internal attributes.
        """

        self.storage_file_path = storage_file_path
        self.storage = None

        self.storage = self._load()

    def __str__(self):
        """
        Returns a string representation of the storage object and the objects contained within it.
        """

        return str(self.storage)

    def store(self, key, data):
        """
        Store `data` under `key`

        Parameters
        ----------
        key: object
            Object that can be used to retrieve the data later. It is recommended that you do not store anything here
            that changes between program runs.
        data: object
            Object that is stored under `key`.

        Notes
        ----------
        The data stored is immediately written to the storage file associated with this object.

        Returns
        -------
        None
        """

        self.storage.update({key: data})
        self._update()
    
    def get(self, key):
        """
        Returns a copy of the object stored under `key`

        Parameters
        ----------
        key: object
            Key associated with the target object.
        
        Returns
        -------
        out: object
        Returns a copy of the object stored under `key`, or None if no object is stored under `key`.
        """
        
        return copy.deepcopy(self.storage.get(key))
    
    def has(self, key):
        """
        Returns whether or not an object with key `key` exists in storage.

        Parameters
        ----------
        key: object
            Key to search for.
        
        Returns
        -------
        out: bool
        Returns True if the key exists in storage, False otherwise.
        """

        return key in self.storage

    def delete(self, key):
        """
        Deletes the key/object pair referenced by `key`, or does nothing if the key is not associated with
        any object in storage.

        Parameters
        ----------
        key: object
            Key associated with the object to be deleted.

        Notes
        ----------
        The new storage object is immediately written to the storage file associated with this object when `key` is found.
        
        Returns
        -------
        out: bool
        Returns True if the function found and sucessfully deleted a key/object pair from storage, False if `key` was not found.
        """

        has_key = self.has(key)

        if has_key:
            del self.storage[key]
            self._update()
        
        return has_key

    def _load(self):
        """
        Retrieves a saved storage object.
        
        Returns
        -------
        out: dict
        Returns the storage object contained within the file at `self.storage_file_path`.
        """

        storage = None

        if not os.path.isfile(self.storage_file_path):
            self._init()
        
        with open(self.storage_file_path, 'rb') as storage_file:
            storage = pickle.load(storage_file)
        
        return storage

    def _init(self):
        """
        Initializes the data storage file with an empty storage object.
        """

        self._empty()
    
    def _empty(self):
        """
        Overwrites the data storage file with an empty storage object.
        """

        self._update({})

    def _update(self, storage=None):
        """
        Updates the data storage file with the latest data storage object.

        Parameters
        ----------
        storage: object
            Data storage object to overwrite the data storage file with.
        
        Returns
        -------
        None
        """

        storage = storage if storage is not None else self.storage

        with open(self.storage_file_path, 'wb') as storage_file:
            pickle.dump(storage, storage_file, protocol=PYTHON_2_COMPATIBLE_PICKLE_PROTOCOL)


def test_global_storage_class(restore_after_test=True):
    """
    Unit test for the `persistence.GlobalStorage` object.
    """

    test_key = 'foo'
    test_value = {'bar'}
    second_test_value = 1/2
    g_storage = GlobalStorage()

    print('Storage at initialization:', g_storage)
    print('Is key "%s" in storage? %s' % (test_key, 'yes' if g_storage.has(test_key) else 'no'))

    g_storage.store(test_key, test_value)
    stored_value = g_storage.get(test_key)
    old_storage_as_str = str(g_storage)

    print('Is key "%s" in storage now? %s' % (test_key, 'yes' if g_storage.has(test_key) else 'no'))
    print('Value associated with key "%s" after attempting to store %s there:' % (test_key, str(test_value)), stored_value)
    print('Storage after storing one value:', g_storage)

    print('Attempting to change the storage through the retrieved stored value...')

    stored_value.update({'baz'})

    print('Storage should still be the same, but is it? %s' % ('yes' if old_storage_as_str == str(g_storage) else 'no',))
    print('stored_value is now', stored_value, 'and storage is', g_storage)

    print('Attempting to update the data associated with "%s" to %s...' % (test_key, str(second_test_value)))
    
    g_storage.store(test_key, second_test_value)

    print('The data associated with "%s" is now' % (test_key,), g_storage.get(test_key))

    if restore_after_test:
        print('Attempting to delete the test key/data pair from storage...')
        g_storage.delete(test_key)
        print('After the attempted deletion, storage is now', g_storage)


if __name__ == '__main__':
    # unit tests
    test_global_storage_class()