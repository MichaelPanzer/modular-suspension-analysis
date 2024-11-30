from independent_suspension import Five_Link
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from matplotlib import rc
from vpython import points, curve, vector, color


#np.set_printoptions(precision=2, suppress=True, linewidth=150)

plt.rcParams['xtick.labelsize']=20      # change the tick label size for x axis
plt.rcParams['ytick.labelsize']=20      # change the tick label size for x axis
plt.rcParams['axes.linewidth']=3        # change the line width of the axis
plt.rcParams['xtick.major.width'] = 3   # change the tick line width of x axis
plt.rcParams['ytick.major.width'] = 3   # change the tick line width of y axis 
rc('text', usetex=False)                # disable LaTeX rendering in plots
rc('font',**{'family':'DejaVu Sans'})   # set the font of the plot to be DejaVu Sans


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
x_0 = np.array([ 5, 0, -1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

z_vals = np.linspace(-1, 1, 200)

positions = create_table(z_vals, solve_z, x_0, fl)


z = positions[:,2]


x = positions[:,0]
y = positions[:,1]

plt.plot(z, x)
plt.plot(z, y)

plt.legend(["x","y"])
plt.show()


caster = positions[:,3]
camber = positions[:,4]
toe = positions[:,5]

plt.plot(z, caster)
plt.plot(z, camber)
plt.plot(z, toe)

plt.legend(["caster", "camber", "toe"])
plt.show()
