import numpy as np
from five_link import Five_Link
from suspension_components import A_Arm


def test_P_W():
    correct_output = np.array([[1,0,0],
                               [0,1,0],
                               [0,0,1],
                               [1,0,0],
                               [0,1,0],
                               [0,0,1],
                               [1,0,0],
                               [0,1,0],
                               [0,0,1],
                               [1,0,0],
                               [0,1,0],
                               [0,0,1],
                               [1,0,0],
                               [0,1,0],
                               [0,0,1]])
    
    fl = Five_Link()
    assert np.array_equal(fl.__gen_P_W__(), correct_output)

def test_P_R():
    upright_pickups = np.array([[1,2,3],
                                [4,5,6],
                                [7,8,9],
                                [10,11,12],
                                [13,14,15]])
    
    correct_output = np.array([[1,0,0,2,0,0,3,0,0],
                               [0,1,0,0,2,0,0,3,0],
                               [0,0,1,0,0,2,0,0,3],
                               [4,0,0,5,0,0,6,0,0],
                               [0,4,0,0,5,0,0,6,0],
                               [0,0,4,0,0,5,0,0,6],
                               [7,0,0,8,0,0,9,0,0],
                               [0,7,0,0,8,0,0,9,0],
                               [0,0,7,0,0,8,0,0,9],
                               [10,0,0,11,0,0,12,0,0],
                               [0,10,0,0,11,0,0,12,0],
                               [0,0,10,0,0,11,0,0,12],
                               [13,0,0,14,0,0,15,0,0],
                               [0,13,0,0,14,0,0,15,0],
                               [0,0,13,0,0,14,0,0,15]])
    fl = Five_Link()

    assert np.array_equal(fl.__gen_P_R__(upright_pickups), correct_output)

def test_P_AB():
    lengths = np.array([21,22,23,24,25])

    correct_output = np.array([[-21,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                               [0,-21,0,0,0,0,0,0,0,0,0,0,0,0,0],
                               [0,0,-21,0,0,0,0,0,0,0,0,0,0,0,0],
                               [0,0,0,-22,0,0,0,0,0,0,0,0,0,0,0],
                               [0,0,0,0,-22,0,0,0,0,0,0,0,0,0,0],
                               [0,0,0,0,0,-22,0,0,0,0,0,0,0,0,0],
                               [0,0,0,0,0,0,-23,0,0,0,0,0,0,0,0],
                               [0,0,0,0,0,0,0,-23,0,0,0,0,0,0,0],
                               [0,0,0,0,0,0,0,0,-23,0,0,0,0,0,0],
                               [0,0,0,0,0,0,0,0,0,-24,0,0,0,0,0],
                               [0,0,0,0,0,0,0,0,0,0,-24,0,0,0,0],
                               [0,0,0,0,0,0,0,0,0,0,0,-24,0,0,0],
                               [0,0,0,0,0,0,0,0,0,0,0,0,-25,0,0],
                               [0,0,0,0,0,0,0,0,0,0,0,0,0,-25,0],
                               [0,0,0,0,0,0,0,0,0,0,0,0,0,0,-25]])
    fl = Five_Link()
    
    assert np.array_equal(fl.__gen_P_AB__(lengths), correct_output)

def test_P():
    upright_pickups = np.array([[1,2,3],
                                [4,5,6],
                                [7,8,9],
                                [10,11,12],
                                [13,14,15]])
    
    lengths = np.array([21,22,23,24,25])

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
 
    fl = Five_Link()
    
    assert np.array_equal(fl.gen_P(upright_pickups, lengths), correct_output)



def test_A():
    frame_pickups = np.array([[31,32,33],
                                [34,35,36],
                                [37,38,39],
                                [40,41,42],
                                [43,44,45]])
    
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

    fl = Five_Link()

    assert fl.gen_A(frame_pickups).all() == correct_output.all()

def test_linear_solve():
    P = np.array([[1,5,1,1,1], 
                  [2,5,1,1,1]])

    A = np.atleast_2d(np.array([5,2])).T

    #driving_variable = "W_y"

    #correct answersf
    correct_upper_matrix = np.array([[2,5, 1,1,1],
                               [0,2.5,0.5,0.5,0.5]])
    #correct_driving_colum = np.atleast_2d(np.array([5, 2.5])).T
    correct_A = np.atleast_2d(np.array([2,4])).T

    fl = Five_Link()

    u, A = fl.solve_linear(P, A)


    assert np.array_equal(A, correct_A) and np.array_equal(u, correct_upper_matrix)
