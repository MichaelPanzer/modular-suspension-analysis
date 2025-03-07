#import importlib
from suspmatics.independent_suspension import Kinematic_Model
import scipy as sp
import numpy as np
from matplotlib import pyplot as plt

link_lengths = np.array([296.3, 326.8, 377.4, 285.0, 526.2])
frame_pickups = np.array([[70.3, 422.1, -68.1],
                          [110.2, 352.8, 98.8],
                          [198.3, 406.9, 116.8],
                          [-196.6, 434.2, -48.8],
                          [-159.2, 194.9, 137]])
upright_pickups = np.array([[-61.9, -105.5, -82.1],
                          [123.5, -117, 90.3],
                          [-9.3, -98.5, 173.3],
                          [-124.4, -86.8, -68.9],
                          [-79.6, -85.7, 141.9]])

fl = Kinematic_Model.five_link(frame_pickups, link_lengths, upright_pickups)

#solves suspension kinematics system of equations in terms of z
def solve_system(driving_args, guess, kinematic_model, method='hybr'): #driving args is tuple of lists
    
    def function(vars):
        return kinematic_model.full_sys_of_eq(vars, driving_args)
    
    def jacobian(vars):
        return kinematic_model.jacobian(vars, driving_args[0])

    return sp.optimize.root(function, guess,jac=jacobian, method=method) 


x_0_lm1 = solve_system(([0,1,2,3,4,5], [0,800,0,0,0,0]), [0,800,0,0,0,0,np.pi/2,0,np.pi/2,0,np.pi/2,0,np.pi/2,0,np.pi/2,0], fl, method='lm')
x_0_lm2 = fl.initial_guess(([0,1,2,3,4,5], [0,800,0,0,0,0]))

x_0_hy1 = solve_system(([2,], [0,]), x_0_lm1.x, fl, method='hybr')
x_0_hy2 = solve_system(([2,], [0,]), x_0_lm2.x, fl, method='hybr')

#difference between two initial guesses
diff = x_0_lm1.x-x_0_lm2.x
print(diff.dot(diff))


#difference between initial guess 2 & its final answer
diff = x_0_hy2.x-x_0_lm2.x
print(diff.dot(diff))

table = fl.create_table(([0,1,2,3,4,5], [0,800,0,0,0,0]), 2, np.linspace(-100, 100))


plt.plot(table[:,2], table[:,4])#z, camber
plt.plot(table[:,2], table[:,3])#z, caster
plt.plot(table[:,2], table[:,5])#z, toe



plt.show()
