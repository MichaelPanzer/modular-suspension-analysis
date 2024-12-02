from suspension_kinematics.components import Single_Link
from vpython import rate
from vpython import sphere
from vpython import color
from vpython import vector
from vpython import arrow


link = Single_Link([0,0,0], 5)

angle = 0
increment = 0.01


arrow(axis=vector(1,0,0), color=color.green)
arrow(axis=vector(0,1,0), color=color.yellow)
arrow(axis=vector(0,0,1), color=color.red)

for i in range(500):
    rate(50)
    link.update_vp_position((angle, .5))
    angle+= increment

