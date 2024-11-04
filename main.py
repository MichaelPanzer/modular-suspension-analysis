import numpy as np
import scipy as sp
import five_link as fl
import sympy 

np.set_printoptions(precision=2, suppress=True, linewidth=150)

link_lengths = np.atleast_2d(np.array([5,5,5,5,5])).T

#this should lead to entirely paralell links
frame_pickups = np.array([[0,1,1],
                          [0,-1,1],
                          [0,-1,-1],
                          [0,1,-1],
                          [0,1,0]])

upright_pickups = np.array([[0,1,1],
                          [0,-1,1],
                          [0,-1,-1],
                          [0,1,-1],
                          [0,1,0]])

P_mat = fl.gen_P(upright_pickups, link_lengths)

A_vec = fl.gen_A(frame_pickups)

x = np.array(sympy.symbols("w_x w_y w_z r_0 r_1 r_2 r_3 r_4 r_5 r_6 r_7 r_8 ab_x_0, ab_y_0, ab_z_0 ab_x_1 ab_y_1 ab_z_1 ab_x_2 ab_y_2 ab_z_2 ab_x_3 ab_y_3 ab_z_3 ab_x_4 ab_y_4 ab_z_4"))

driving_expression = x[2] - 0


def equations(vars):
    return fl.equations(vars, P_mat, A_vec, driving_expression)

x_0 = np.array([5,0,0,
                1,0,0,
                0,1,0,
                0,0,1,
                1,0,0,
                1,0,0,
                1,0,0,
                1,0,0,
                1,0,0])


#print(equations(x))

sol = sympy.nsolve(equations(x), x,  x_0)

#sol = equations(x_0)
print(sol)

