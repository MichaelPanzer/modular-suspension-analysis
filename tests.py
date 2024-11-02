import numpy as np
from Five_Link import __block__


def test_block_square():
    block_1 = np.array([[1,2], [5,6]])
    block_2 = np.array([[3,4], [7,8]])
    block_3 = np.array([[9,10], [13,14]])
    block_4 = np.array([[11,12], [15,16]])

    input_mat = np.array([[block_1, block_2], [block_3, block_4]])

    output = __block__(input_mat)

    correct_output = np.array([[1,2,3,4],
                               [5,6,7,8],
                               [9,10,11,12],
                               [13,14,15,16]])
    
    assert output.all()==correct_output.all()

def test_block_non_square():
    block_1 = np.array([[1,2], [10,11]])
    block_2 = np.array([[3,4,5], [12,13,14]])
    block_3 = np.array([[6,7,8,9], [15,16,17,18]])


    input_mat = np.array([block_1, block_2, block_3])

    output = __block__(input_mat)

    correct_output = np.array([[1,2,3,4,5,6,7,8,9],
                               [10,11,12,13,14,15,16,17,18]])
    
    assert output.all()==correct_output.all()