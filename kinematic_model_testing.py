from independent_suspension import Five_Link
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from vpython import points, curve, vector, color

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


fl = Five_Link(frame_pickups, link_lengths, upright_pickups)

#print(equations(x, x[2]-69))

#solves suspension kinematics system of equations in terms of z
def solve_z(z, guess, system):
    driving_var = 2
    
    def function(vars):
        return system.full_sys_of_eq(vars, driving_var, z)

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

z_vals = np.linspace(-2, 2, 3)

positions = create_table(z_vals, solve_z, x_0, fl)


print(positions)

def draw_sus(x, A):
    wheel = vector(x[0], x[1], x[2])
    frame_pickup_0 = vector(frame_pickups[0][0], frame_pickups[0][1], frame_pickups[0][2])
    frame_pickup_1 = vector(frame_pickups[1][0], frame_pickups[1][1], frame_pickups[1][2])
    frame_pickup_2 = vector(frame_pickups[2][0], frame_pickups[2][1], frame_pickups[2][2])
    frame_pickup_3 = vector(frame_pickups[3][0], frame_pickups[3][1], frame_pickups[3][2])
    frame_pickup_4 = vector(frame_pickups[4][0], frame_pickups[4][1], frame_pickups[4][2])

    

    points(pos=[wheel, frame_pickups, ], color=color.red)

draw_sus(positions[0], frame_pickups)

input('Press ENTER to exit')