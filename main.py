import numpy as np
import five_link as fl
np.set_printoptions(precision=2, suppress=True, linewidth=150)

link_lengths = np.array([5,5,5,5,5]).T

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
P_inv = np.linalg.pinv(P)

A_vec = fl.gen_A(frame_pickups)

sol = np.dot(P_inv, A_vec.T)


print(P)
print("\n\n")
print(sol)