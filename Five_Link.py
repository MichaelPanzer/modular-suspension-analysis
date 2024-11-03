import numpy as np
import scipy as sp
import sympy as symp


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


""""
def gen_AB_nonlinear_eq(vars):
    AB_x, AB_y, AB_z = vars

    beta = np.arccos(AB_z)
    alpha = np.arcsin(AB_y/np.sin(beta))

    AB_x = np.cos(alpha)*np.sin(beta)
    return AB_x, AB_y, AB_z


def gen_R_nonlinear_eq(vars):
    r_0,r_1,r_2,r_3,r_4,r_5,r_6,r_7,r_8 = vars

    phi = np.arcsin(-1*r_2)
    theta = np.arcsin(r_5/np.cos(phi))
    gamma = np.arcsin(r_2/np.cos(phi))

    r_0 = np.cos(phi)*np.cos(gamma)
    r_1 = np.cos(phi)*np.sin(gamma)

    r_3
    r_4 = np.sin(theta)*np.sin(phi)*np.sin(gamma) + 
"""
def gen_AB_nonlinear_eq(vars):
    AB_x, AB_y, AB_z = vars

    return AB_x**2 + AB_y**2 + AB_z**2 - 1


def gen_R_nonlinear_eq(vars):
    r_0,r_1,r_2,r_3,r_4,r_5,r_6,r_7,r_8 = vars

    eq_0 = r_0**2 + r_3**2 + r_6**2 - 1
    eq_1 = r_1**2 + r_4**2 + r_7**2 - 1
    eq_2 = r_2**2 + r_5**2 + r_8**2 - 1

    return np.array([eq_0, eq_1, eq_2])


def equations(x, P, A):
    
    linear_equations = (np.dot(P, x)).T - A.T
     
    r_vec = x[3:12]
    eq_r_0, eq_r_1, eq_r_2 = gen_R_nonlinear_eq(r_vec)

    r_eqs = np.array([eq_r_0, eq_r_1, eq_r_2])

    ab_eqs = np.zeros(5, dtype=object)
    ab_vecs = x[12:]

    for i, eq in enumerate(ab_eqs):
        ab_eqs[i] = gen_AB_nonlinear_eq(ab_vecs[i:i+3])
        print(ab_eqs[i])
    
    return np.concatenate((linear_equations[0], r_eqs, ab_eqs))

