from suspension_kinematics.components import Upright
from vpython import rate
from vpython import sphere
from vpython import color
from vpython import vector
from vpython import arrow
import numpy as np

upright_pickups = np.array([[0.9,0,0.9],
                          [-0.9,0,0.9],
                          [-0.9,0,-0.9],
                          [0.9,0,-0.9],
                          [1.5,0,0]])

upright = Upright(upright_pickups)

angle = 0
increment = 0.01

arrow(axis=vector(0,0,1), color=color.blue)
arrow(axis=vector(-1,0,0), color=color.green)
arrow(axis=vector(0,-1,0), color=color.red)

for i in range(500):
    rate(6)
    upright.update_vp_position((1,0,0, 0, 0, 0))
    angle+= increment

