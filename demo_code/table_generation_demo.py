#import importlib
from suspmatics.independent_suspension import Kinematic_Model# type: ignore
from suspmatics.visuals import Renderer
import scipy as sp# type: ignore
import numpy as np
from matplotlib import pyplot as plt# type: ignore

#These dimensions roughly represent the rear suspension of an NC Miata
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


initial_pos = [(0,0.), (1,800), (2,0), (3,0), (4,0), (5,0)] #these are the positions used to create the initial guess
driving_var = 2 #index of z variable
driving_values = np.linspace(-100, 100) #list of z values used to create the table

table = fl.create_table(initial_pos, driving_var, driving_values)


plt.plot(table[:,2], table[:,4])#z, camber
plt.plot(table[:,2], table[:,3])#z, caster
plt.plot(table[:,2], table[:,5])#z, toe

plt.show()



r = Renderer(fl.components)

r.sae_basis_vectors()
r.animate(table)