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

#solves suspension kinematics system of equations in terms of z
def solve_z(z, guess, system):
    driving_var = 2
    
    def function(vars):
        return system.gen_systems_of_equations(vars, driving_var, z)

    return sp.optimize.fsolve(function, guess)

#solves for a list of inputs, using each output as a guess for the next solution
def create_table(inputs, solver, inital_guess, system):
    guess = inital_guess
    
    outputs = np.zeros([len(inputs),len(inital_guess)])

    for i, input in enumerate(inputs):
        outputs[i] = solver(input, guess, system)
        guess = outputs[i]

    return outputs


#creates a table of solutions which can be plotted and analyized 
x_0 = np.array([ 4.48, -0.94, -2.,   -0.01, -0.05,  0.03, -0.23, -0.44, -0.18, -0.43, -0.18, -0.39, -0.23, -0.39, -0.23, -0.41])

z_vals = np.linspace(-2, 2, 10)

positions = create_table(z_vals, solve_z, x_0, five_link)


print(positions)

