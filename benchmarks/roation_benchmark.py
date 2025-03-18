#type: ignore
import numpy as np
import time
from scipy.spatial.transform import Rotation

#creates rot matrices for each axis and multiplies them together
def ver_a(vec: np.ndarray, angles: tuple):
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

#same as ver a except the trig functions are calculated once and stored as vars
def ver_b(vec: np.ndarray, angles: tuple):
    phi, theta, gamma = angles
    ct = np.cos(theta)
    st = np.sin(theta)

    cp = np.cos(phi)
    sp = np.sin(phi)

    cg = np.cos(gamma)
    sg = np.sin(gamma)
    
    r_x = np.array([[1, 0, 0],
        [0, ct, -st],
        [0, st, ct]])
        
    r_y = np.array([[cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp]])
        
    r_z = np.array([[cg, -sg, 0],
        [sg, cg, 0],
        [0, 0, 1]])
    
    return vec.dot(r_x.dot(r_y.dot(r_z)))

#using scipy rotation class
def ver_c(vec: np.ndarray, angles: tuple):
    r = Rotation.from_euler("xyz", angles).as_matrix()
    
    return vec.dot(r)


count = 10**6

random_inputs = np.random.rand(count, 2, 3)

#ver a
start_time = time.time()
for input in random_inputs:
    output = ver_a(input[0], input[1])
end_time = time.time()
time_a = end_time-start_time
print("a: " + str(time_a))

#ver b
start_time = time.time()
for input in random_inputs:
    output = ver_b(input[0], input[1])
end_time = time.time()
time_b = end_time-start_time
print("b: " + str(time_b))

#ver c
start_time = time.time()
for input in random_inputs:
    output = ver_c(input[0], input[1])
end_time = time.time()
time_c = end_time-start_time
print("c: " + str(time_c))


def as_perc(datum, value):
    return 100*(value/datum)


print(as_perc(time_a, time_a))
print(as_perc(time_a, time_b))
print(as_perc(time_a, time_c))