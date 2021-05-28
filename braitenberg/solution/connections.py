from typing import Tuple

import numpy as np


# NOTE: these strategies attempt to avoid the ducks

def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")
    res[:, -shape[1]//2:] = -1
    res *= np.expand_dims(np.linspace(0, 1, num=shape[0]), axis=-1)
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")
    res[:, :-shape[1]//2] = -1
    res *= np.expand_dims(np.linspace(0, 1, num=shape[0]), axis=-1)
    return res