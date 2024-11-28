import numpy as np
from suspension_components import *
from scipy.linalg import block_diag


class Kinematic_Model:

    def __init__(self, linkages, wheel_carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier
        #self.linear_system, self.frame_pickups = self.__generate_linear_system__()

    def from_file(self, file_name):
        pass

    def global_A_matrix(self):
        wheelcarrier_local_A = self.wheel_carrier.local_A_matrix()
        link_local_A = np.zeros(self.linkages.size, dtype=np.ndarray)

        for i, linkage in enumerate(self.linkages):
            link_local_A[i] = linkage.local_A_matrix()

        A_links = block_diag(*link_local_A)


        A_matrix = np.block([wheelcarrier_local_A, A_links])
        
        
        return A_matrix
    
    def global_B_vector(self):
        B_vector = np.zeros(self.linkages.size, dtype=np.ndarray)

        for i, linkage in enumerate(self.linkages):
            B_vector[i] = linkage.local_B_vector()
        
        return np.atleast_2d(np.block(B_vector.tolist())).T

    def generate_x_nonlinear(self, vars):
        x = np.zeros(27) # fix this so it isnt stupid


        x[0:12] = self.wheel_carrier.nonlin_x_expression(vars[0:6]) #wheel carrier position and rotation

        link_vecs = x[12:]
        link_angles = vars[6:]

        #This is really bad
        i = 0 #link vec index
        j = 0 #link angle index
        for linkage in self.linkages:
            num_nonlin_outputs = 3
            num_nonlin_inputs = 2 #TODO make this dependent on the link
                              
            link_vecs[i:i+num_nonlin_outputs] = linkage.nonlin_x_expression(link_angles[j:j+num_nonlin_inputs])

            i += num_nonlin_outputs
            j += num_nonlin_inputs

        return x
    
    def full_sys_of_eq(self, vars, driving_var, value):
        x = self.generate_x_nonlinear(vars)

        nonlin_expressions = (np.dot(self.global_A_matrix(), x).T - self.global_B_vector().T)[0]

        #print(nonlin_expressions)

        driving_expression = x[driving_var] - value

        return np.concatenate((nonlin_expressions, np.array([driving_expression])))





    
    def render(self):
        pass


#this class is mainly for testing functionality before a more universal class can be made
class Five_Link(Kinematic_Model):
    def __init__(self, frame_pickups, link_lengths, upright_pickups):
        linkages = np.zeros(5, dtype=Linkage)

        for i, link in enumerate(linkages):
            linkages[i] = Single_Link(frame_pickups[i], link_lengths[i])

        upright = Upright(upright_pickups)

        super(Five_Link, self).__init__(linkages, upright)


    

        
    