import numpy as np
import scipy as sp
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

A_vec = fl.gen_A(frame_pickups)

p, l, u = sp.linalg.lu(P)

print(p)
print("\n\n")

print(l)
print("\n\n")

print(u)
print("\n\n")
#print("\n\n")
