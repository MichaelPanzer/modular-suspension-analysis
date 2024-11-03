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
    print(output)
    return np.block(output.tolist())

def __gen_P_AB__(lengths):
    output = np.zeros(lengths.size * 3)
    #output = [] * (lengths.size*3)

    for i, length in enumerate(lengths):
        for j in range(3*i, 3*i+3):
            output[j] = -1*length

    print()
    return np.diag(output)

def gen_P(upright_pickups, lengths):
    return np.block([__gen_P_W__(), __gen_P_R__(upright_pickups), __gen_P_AB__(lengths)])

def gen_A(frame_pickups):    
    return np.ravel(frame_pickups).T


