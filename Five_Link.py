import numpy as np


#figure out a better way to do this np.block?
def __block__(matrix):
    block_shape = matrix[0][0].shape
    output = np.zeros((matrix.shape[0]*block_shape[0], matrix.shape[1]*block_shape[1]))

    for i, row in enumerate(matrix):
        for j, block in enumerate(row):
            for k, block_row in enumerate(block):
                for l, val in enumerate(block_row):
                    output[i*block_shape[0]+k][j*block_shape[1]+l] = val

    return output


def __gen_P_R__(upright_pickups):
    p_r = np.zeros((5,3), dtype=np.ndarray)
    


    for i, pickup_n in enumerate(upright_pickups) :
        for j, pickup_coord in enumerate(pickup_n):
            p_r[i, j] = pickup_coord*np.identity(3)

    return __block__(p_r)


mat = __gen_P_R__(np.array([[1,2,3],
                    [1,2,3],
                    [1,2,3],
                    [1,2,3],
                    [1,2,3]]))

print(mat.shape)

print("\n\n\n")
print(mat)

#block1 = np.array([[1,2], [3,4]])
#block2 = np.array([[5,6], [7,8]])


#print(mat)
#print("\n\n")
#print(np.concatenate(mat, axis = 1))
        
