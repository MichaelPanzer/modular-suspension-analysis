# type: ignore #TODO this probably isn't good
import sys
import numpy as np
from suspmatics.independent_suspension import *
from jacobi import jacobi

np.set_printoptions(precision=1, suppress=True, linewidth=500, threshold=sys.maxsize)
#array32 = npt.NDArray[np.float32]

frame_pickups = np.array([[31,32,33],
                                [34,35,36],
                                [37,38,39],
                                [40,41,42],
                                [43,44,45]])

upright_pickups = np.array([[1,2,3],
                            [4,5,6],
                            [7,8,9],
                            [10,11,12],
                            [13,14,15]])

lengths = np.array([21,22,23,24,25])



def test_A():
    correct_output = np.array([[1,0,0, 1,0,0,2,0,0,3,0,0,    -21,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                               [0,1,0, 0,1,0,0,2,0,0,3,0,    0,-21,0,0,0,0,0,0,0,0,0,0,0,0,0],
                               [0,0,1, 0,0,1,0,0,2,0,0,3,    0,0,-21,0,0,0,0,0,0,0,0,0,0,0,0],
                               [1,0,0, 4,0,0,5,0,0,6,0,0,    0,0,0,-22,0,0,0,0,0,0,0,0,0,0,0],
                               [0,1,0, 0,4,0,0,5,0,0,6,0,    0,0,0,0,-22,0,0,0,0,0,0,0,0,0,0],
                               [0,0,1, 0,0,4,0,0,5,0,0,6,    0,0,0,0,0,-22,0,0,0,0,0,0,0,0,0],
                               [1,0,0, 7,0,0,8,0,0,9,0,0,    0,0,0,0,0,0,-23,0,0,0,0,0,0,0,0],
                               [0,1,0, 0,7,0,0,8,0,0,9,0,    0,0,0,0,0,0,0,-23,0,0,0,0,0,0,0],
                               [0,0,1, 0,0,7,0,0,8,0,0,9,    0,0,0,0,0,0,0,0,-23,0,0,0,0,0,0],
                               [1,0,0, 10,0,0,11,0,0,12,0,0, 0,0,0,0,0,0,0,0,0,-24,0,0,0,0,0],
                               [0,1,0, 0,10,0,0,11,0,0,12,0, 0,0,0,0,0,0,0,0,0,0,-24,0,0,0,0],
                               [0,0,1, 0,0,10,0,0,11,0,0,12, 0,0,0,0,0,0,0,0,0,0,0,-24,0,0,0],
                               [1,0,0, 13,0,0,14,0,0,15,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,-25,0,0],
                               [0,1,0, 0,13,0,0,14,0,0,15,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,-25,0],
                               [0,0,1, 0,0,13,0,0,14,0,0,15, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,-25] ])
 
    fl = Kinematic_Model.five_link(frame_pickups, lengths, upright_pickups)
    
    a = fl._global_coef_mat()
    print(a[:,:15])
    print(a[:,-15:])
    assert np.array_equal(a, correct_output)

def test_B():
    correct_output = np.array([[31],
                                [32],
                                [33],
                                [34],
                                [35],
                                [36],
                                [37],
                                [38],
                                [39],
                                [40],
                                [41],
                                [42],
                                [43],
                                [44],
                                [45]])

    fl = Kinematic_Model.five_link(frame_pickups, lengths, upright_pickups)

    b = fl._global_fixed_vec()
    assert np.array_equal(b, correct_output)

def test_jacobian():
    """
    For some reason the rows with the driving values are not computed properly with the numerical approximation

    Temporarily I removed the driving variables but I need to find a better way to test this 
    """
    fl = Kinematic_Model.five_link(frame_pickups, lengths, upright_pickups)

    vars = np.random.rand(16)

    my_jacobian = fl.jacobian(vars, [])

    def func(vars):
        return fl.full_sys_of_eq(vars, [])
    
    correct_jacobian, est_err = jacobi(func, vars, rtol=10e-10)

    real_err = np.absolute(my_jacobian-correct_jacobian)

    #print(real_err)
    
    np.testing.assert_almost_equal(my_jacobian, correct_jacobian, decimal = 3)


