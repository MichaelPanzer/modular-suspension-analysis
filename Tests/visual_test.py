# 
# VPython Example 101 - Spheres
#
# Display 3 spheres of different colors on each axis
#
# Author: Alan Badillo Salas <alan@nomadacode.com>
# Created: 2023/02/15
#

from vpython import *

ball1 = sphere(pos=vector(1, 0, 0), radius=0.1, color=color.red)

print(ball1.pos)

ball2 = sphere(pos=vector(0, 1, 0), radius=0.1, color=color.green)

print(ball2.pos)

ball3 = sphere(pos=vector(0, 0, 1), radius=0.1, color=color.blue)

print(ball3.pos)

input('Press ENTER to exit')