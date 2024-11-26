import numpy as np
from suspension_components import *


class Kinematic_Model:

    def __init__(self, linkages, wheel_carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier
        #self.linear_system, self.frame_pickups = self.__generate_linear_system__()

    def from_file(self, file_name):
        pass

    def gen_A_and_B(self):
        A_wheel, A_upright_pickups = self.wheel_carrier.wheel_position_matrix()
        link_local_A_vector = np.array(self.linkages.local_A()) #???
        A_matrix = np.block(A_wheel, A_upright_pickups, A_links)
        
        return A_matrix, np.array([0])
    

    def __generate_nonlinear_equations__(self):
        pass
    
    def render(self):
        pass


#this class is mainly for testing functionality before a more universal class can be made
class Five_Link(Kinematic_Model):
    def __init__(self, frame_pickups, link_lengths, upright_pickups):
        linkages = np.zeros(5, dtype=Linkage)

        for i, link in enumerate(linkages):
            linkages[i] = Single_Link(frame_pickups[i], link_lengths[i])

        upright = Upright(upright_pickups)
        super.__init__(linkages, upright)


    

        
    