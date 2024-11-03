import numpy as np
import scipy as sp


def __gen_P_W__():
    #output = np.zeros((5,1), dtype=np.ndarray)

    output = [[np.identity(3)]*1]*5

    #for i, output_val in enumerate(output):
        #output[i, 0] = -1*np.identity(3)

    return np.block(output)

def __gen_P_R__(upright_pickups):
    output = np.zeros((5,3), dtype=np.ndarray)
    #output = [[np.identity(3)] * 3] *5

    #each block is a pickup coordinate * the identity matrix
    for i, pickup_n in enumerate(upright_pickups) :
        for j, pickup_coord in enumerate(pickup_n):
            #print(output[i, j])
            output[i, j] = pickup_coord*np.identity(3)

    #combines the block matrix
    return np.block(output.tolist())

def __gen_P_AB__(lengths):
    output = np.zeros(lengths.size * 3)
    #output = [] * (lengths.size*3)

    for i, length in enumerate(lengths):
        for j in range(3*i, 3*i+3):
            output[j] = -1*length
    # swap colums arorund
    return np.diag(output)

def gen_P(upright_pickups, lengths):
    return np.block([__gen_P_W__(), __gen_P_R__(upright_pickups), __gen_P_AB__(lengths)])

def gen_A(frame_pickups):    
    return np.atleast_2d(np.ravel(frame_pickups)).T

variable_index_lookup = {"W_x": 0,
                         "W_y": 1,
                         "W_z": 2 }

def solve_linear(P, A):
    #driving_variable = variable_index_lookup[driving_variable]

    #saves then copies the colum of the driving variable from the equation matrix
    #driving_colum = np.array(P[:, [driving_variable]])
    #P = np.delete(P, driving_variable, 1)

    l, u = sp.linalg.lu(P, permute_l=True, overwrite_a=True)
    l = np.linalg.inv(l)

    A = np.dot(l, A)
    #driving_colum = np.dot(l, driving_colum)

    #print(l, u, A)
    return u, A

