from suspension_kinematics.independent_suspension import Five_Link
from sympy import symbols
from sympy import solve_poly_system
from sympy import solve_triangulated

import numpy as np

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

fl = Five_Link(frame_pickups, link_lengths, upright_pickups)

n=16
x = symbols("x_0:{}".format(n))
#print(x)

approx_sys = fl.approx_sys_of_eq(x, 2, 100)

x_0 = solve_poly_system(approx_sys)

print(x_0)

