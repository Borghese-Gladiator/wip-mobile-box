import numpy as np


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
