import numpy as np
from suspension_components import *


class Kinematic_Model:
    def __init__(self, linkages, wheel_carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier

        #check mobility and thow error if not properly defined

        self.linear_system, self.frame_pickups = self.__generate_linear_system__()

    @classmethod
    def from_file(self, file_name):
        pass
    
    @classmethod
    def __generate_linear_system__(self):
        pass
    
    @classmethod
    def __generate_nonlinear_equations__(self):
        pass
    
    @classmethod
    def full_system_of_equations(self, driving_var, driving_value):
        pass

    @classmethod
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
    
    #override
    def generate_linear_system():
        A_wheel, A_upright_pickups = super.wheel_carrier.wheel_position_matrix()
        A_links = ... #TODO add loop to generate vector of link equations
        A_matrix = np.block(A_wheel, A_upright_pickups, A_links)
        
    