import numpy as np
from scipy.spatial.transform import Rotation as R
import pytest

def test_sp_rotation():
    theta, phi, gamma = np.random.rand(3)


    r_x = np.array([[1, 0, 0],
                        [0, np.cos(theta), -1*np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])
    r_y = np.array([[np.cos(phi), 0, np.sin(phi)],
                    [0, 1, 0],
                    [-1*np.sin(phi), 0, np.cos(phi)]])
    r_z = np.array([[np.cos(gamma), -1*np.sin(gamma), 0],
                    [np.sin(gamma), np.cos(gamma), 0],
                    [0, 0, 1]])

    r = (r_x.dot(r_y).dot(r_z))

    print(r)
    print(R.from_euler('XYZ', [theta, phi, gamma]).as_matrix())

    np.testing.assert_almost_equal(r,R.from_euler('XYZ', [theta, phi, gamma]).as_matrix())


