import numpy as np
import time

def method_a(vec: np.ndarray, angles: tuple) -> np.ndarray:
    phi, theta, gamma = angles

    r_x = np.array([[1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]])
        
    r_y = np.array([[np.cos(phi), 0, np.sin(phi)],
        [0, 1, 0],
        [-np.sin(phi), 0, np.cos(phi)]])
        
    r_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]])
    
    return vec.dot(r_x.dot(r_y.dot(r_z)))


count = 10**5

