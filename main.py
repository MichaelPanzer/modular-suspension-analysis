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
"""
upright_pickups = np.array([[0,1,1],
                          [0,-1,1],
                          [0,-1,-1],
                          [0,1,-1],
                          [0,1,0]])
"""
upright_pickups = np.array([[0,0.9,0.9],
                          [0,-0.9,0.9],
                          [0,-0.9,-0.9],
                          [0,0.9,-0.9],
                          [0,0.9,0]])


five_link = fl.Five_Link(link_lengths, frame_pickups, upright_pickups)

#print(equations(x, x[2]-69))

def solution(z, system):
    x_0 = np.array([5,0,0,
                0,0,0,
                0.00,0.00,
                0.00,0.00,
                0.00,0.00,
                0.00,0.00,
                0.00,0.00])
    
    driving_var = 2
    
    def system(vars):
        return five_link.gen_systems_of_equations(vars, driving_var, z)

    return sp.optimize.fsolve(system, x_0)

x = solution(1, five_link)

#converts radian output to degrees
for i, angle  in enumerate(x[3:]):
    x[i+3] = math.degrees(angle)%360

print(x)

#z_vals = np.linspace(-2, 2, 10)
