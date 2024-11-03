import numpy as np
import scipy as sp
import five_link as fl
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

P = fl.gen_P(upright_pickups, link_lengths)

A_vec = fl.gen_A(frame_pickups)

u, driv_col, A = fl.solve_linear("W_z",P, A_vec)


u_w = u[:, 0:2]
print(u_w)

u_r = u[:, 2:11]
print(u_r)

u_a = u[:, 11:]
print(u_a)