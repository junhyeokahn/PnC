import os

import numpy as np
import pickle


class MetaSingleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(MetaSingleton,
                                        cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class DataSaver(metaclass=MetaSingleton):
    """
    Data Saver:
        add topics --> advance
    """
    def __init__(self, filename='pnc.pkl'):
        self._history = dict()
        if not os.path.exists('experiment_data'):
            os.makedirs('experiment_data')
        for f in os.listdir('experiment_data'):
            if f == filename:
                os.remove('experiment_data/' + f)
        self._file = open('experiment_data/' + filename, 'ab')

    def add(self, key, value):
        self._history[key] = value

    def advance(self):
        pickle.dump(self._history, self._file)

    def close(self):
        self._file.close()

    @property
    def history(self):
        return self._history

    @history.setter
    def history(self, value):
        self._history = value
