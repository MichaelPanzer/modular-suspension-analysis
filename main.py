import numpy as np
import scipy as sp
import five_link as fl
import sympy 
import matplotlib.pyplot as plt
import math

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

#x = np.array(sympy.symbols("w_x w_y w_z r_0 r_1 r_2 r_3 r_4 r_5 r_6 r_7 r_8 ab_x_0, ab_y_0, ab_z_0 ab_x_1 ab_y_1 ab_z_1 ab_x_2 ab_y_2 ab_z_2 ab_x_3 ab_y_3 ab_z_3 ab_x_4 ab_y_4 ab_z_4"))
#x = np.array(sympy.symbols("w_x w_y w_z theta phi gamma alpha_0 beta_0 alpha_1 beta_1 alpha_2 beta_2 alpha_3 beta_3 alpha_4 beta_4"))




#print(equations(x, x[2]-69))

def solution(z, P, A):
    x_0 = np.array([0,0,0,
                0,0,0,
                0.01,0.01,
                0.01,-0.01,
                -0.01,0.01,
                -0.1,-0.1,
                0.1,0.1])
    
    driving_var = 2
    
    def system(vars):
        return fl.gen_systems_of_equations(vars, P, A, driving_var, z)

    return sp.optimize.fsolve(system, x_0)

x = solution(0.1, P_mat, A_vec)

#converts radian output to degrees
for i, angle  in enumerate(x[2:]):
    x[i+2] = math.degrees(angle)%360

print(x)

#z_vals = np.linspace(-2, 2, 10)

"""
x_vals = w_x(z_vals)

print(z_vals)
print(x_vals)

plt.plot(z_vals, x_vals)
plt.show()
"""
