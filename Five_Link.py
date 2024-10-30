import numpy as np
import scipy as sp


#figure out a better way to do this np.block?
def __block__(matrix):

    #sets size of output matrix
    block_shape = matrix[0][0].shape
    output = np.zeros((matrix.shape[0]*block_shape[0], matrix.shape[1]*block_shape[1]))

    #iterates through the input matrix 
    for i, row in enumerate(matrix):
        for j, block in enumerate(row):
            #iterates through each block of the input matrix 
            for k, block_row in enumerate(block):
                for l, val in enumerate(block_row):
                    output[i*block_shape[0]+k][j*block_shape[1]+l] = val

    return output


def __gen_P_R__(upright_pickups):
    p_r = np.zeros((5,3), dtype=np.ndarray)

    #each block is a pickup coordinate * the identity matrix
    for i, pickup_n in enumerate(upright_pickups) :
        for j, pickup_coord in enumerate(pickup_n):
            p_r[i, j] = pickup_coord*np.identity(3)

    #combines the block matrix
    return __block__(p_r)

def __gen_P_AB__(lengths):
    output = np.zeros(lengths.size * 3)

    for i, length in enumerate(lengths):
        for j in range(3*i, 3*i+3):
            output[j] = length

    return np.diag(output)


mat = __gen_P_R__(np.array([[1,2,3],
                    [1,2,3],
                    [1,2,3],
                    [1,2,3],
                    [1,2,3]]))



#print(mat)

ab_mat = __gen_P_AB__(np.array([1,2,3,4,5]))

print(ab_mat)


