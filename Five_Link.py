import numpy as np
import scipy as sp
import sympy 


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



def gen_AB_nonlinear_eq(vars):
    alpha, beta = vars
 
    AB_x = sympy.cos(alpha) * sympy.sin(beta)
    AB_y = sympy.sin(alpha) * sympy.sin(beta)
    AB_z = sympy.cos(beta)
    
    return AB_x, AB_y, AB_z


def gen_R_nonlinear_eq(vars):
    theta, phi, gamma = vars

    r_x = np.array([[1, 0, 0],
                    [0, sympy.cos(theta), -1*sympy.sin(theta)],
                    [0, sympy.sin(theta), sympy.cos(theta)]])
    r_y = np.array([[sympy.cos(phi), 0, sympy.sin(phi)],
                    [0, 1, 0],
                    [-1*sympy.sin(phi), 0, sympy.cos(phi)]])
    r_z = np.array([[sympy.cos(gamma), -1*sympy.sin(gamma), 0],
                    [sympy.sin(gamma), sympy.cos(gamma), 0],
                    [0, 0, 1]])

    r = r_x.dot(r_y).dot(r_z)

    return r.T.flatten()

"""
def gen_AB_nonlinear_eq(vars):
    AB_x, AB_y, AB_z = vars

    return AB_x**2 + AB_y**2 + AB_z**2 - 1


def gen_R_nonlinear_eq(vars):
    r_0,r_1,r_2,r_3,r_4,r_5,r_6,r_7,r_8 = vars


    #this comes from the R*Rt = I
    eq_0 = r_0**2 + r_3**2 + r_6**2 - 1
    eq_1 = r_1**2 + r_4**2 + r_7**2 - 1
    eq_2 = r_2**2 + r_5**2 + r_8**2 - 1

    eq_3 = r_0*r_1 + r_3*r_4 + r_6*r_7
    eq_4 = r_0*r_2 + r_3*r_5 + r_6*r_8
    eq_5 = r_1*r_2 + r_4*r_5 + r_7*r_8

    return np.array([eq_0, eq_1, eq_2, eq_3, eq_4, eq_5])
"""

def gen_systems_of_equations(x, P, A, driving_var, value):
    #x_linear = np.array(sympy.symbols("w_x w_y w_z r_0 r_1 r_2 r_3 r_4 r_5 r_6 r_7 r_8 ab_x_0, ab_y_0, ab_z_0 ab_x_1 ab_y_1 ab_z_1 ab_x_2 ab_y_2 ab_z_2 ab_x_3 ab_y_3 ab_z_3 ab_x_4 ab_y_4 ab_z_4"))
    x_linear = np.zeros(27)
    
    x_linear[0:3] = x[0:3]

    x_linear[3:12] = gen_R_nonlinear_eq(x[3:6])
    #print(x_linear)
    #print("\n")

    ab_vecs = x_linear[12:]
    ab_angles = x[6:]

    for i in range(5):
        ab_vecs[3*i:3*i+3] = gen_AB_nonlinear_eq((ab_angles[2*i], ab_angles[2*i+1]))
    
    #print(x_linear)
    #print("\n")

    
    linear_equations = ((np.dot(P, x_linear)).T - A.T)[0]

    driving_expression = x[driving_var] - value
    
    return np.concatenate((linear_equations, np.array([driving_expression])))

