import numpy as np
import scipy as sp
import five_link as fl
import sympy as sp

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
""""
u, driv_col, A = fl.solve_linear("W_z",P, A_vec)


u_w = u[:, 0:2]
print(u_w)

u_r = u[:, 2:11]
print(u_r)

u_a = u[:, 11:]
print(u_a)
"""

x = np.array(sp.symbols("w_x w_y w_z r_0 r_1 r_2 r_3 r_4 r_5 r_6 r_7 r_8 ab_x_0, ab_y_0, ab_z_0 ab_x_1 ab_y_1 ab_z_1 ab_x_2 ab_y_2 ab_z_2 ab_x_3 ab_y_3 ab_z_3 ab_x_4 ab_y_4 ab_z_4"))

system = fl.equations(x, P_mat, A_vec)

print(system)
"""
print(linear_equations.shape)
print("\n\n\n")


print(r_eqs)
print("\n\n\n")


print(ab_eqs)
print("\n\n\n")
"""