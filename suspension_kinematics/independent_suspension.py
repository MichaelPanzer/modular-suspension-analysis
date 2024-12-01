import numpy as np
from suspension_kinematics.components import *
from scipy.linalg import block_diag
from scipy.linalg import lu

class Kinematic_Model:

    def __init__(self, linkages, wheel_carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier

        self.a_mat = self.global_A_matrix()
        self.b_vec = self.global_B_vector()
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
    
    def approx_x_nonlinear(self, vars):
        x = np.zeros(27, dtype=type(vars[0])) # fix this so length is dynamic

        x[0:12] = self.wheel_carrier.approx_nonlin_x(vars[0:6]) #wheel carrier position and rotation

        link_vecs = x[12:]
        link_angles = vars[6:]

        #TODO This is really bad
        i = 0 #link vec index
        j = 0 #link angle index
        for linkage in self.linkages:
            num_nonlin_outputs = 3
            num_nonlin_inputs = 2 #TODO make this dependent on the link
                              
            link_vecs[i:i+num_nonlin_outputs] = linkage.approx_nonlin_x(link_angles[j:j+num_nonlin_inputs])

            i += num_nonlin_outputs
            j += num_nonlin_inputs

        return x
    
    def full_sys_of_eq(self, vars, driving_var, value):
        x = self.generate_x_nonlinear(vars)

        nonlin_expressions = (np.dot(self.a_mat, x).T - self.b_vec.T)[0]

        driving_expression = x[driving_var] - value

        return np.concatenate((nonlin_expressions, np.array([driving_expression])))

    def approx_sys_of_eq(self, vars, driving_var, value):
            x = self.approx_x_nonlinear(vars)

            nonlin_expressions = (np.dot(self.a_mat, x).T - self.b_vec.T)[0]

            #print(nonlin_expressions)

            driving_expression = x[driving_var] - value

            return np.concatenate((nonlin_expressions, np.array([driving_expression])))
    

    #TODO write some fucking tests stoopidhead
    def jacobian(self, vars, driving_var):
        jacobians = np.zeros(self.linkages.shape[0] + 1, dtype=np.ndarray)
        link_angles = vars[6:]
        jacobians[0] = self.wheel_carrier.jacobian(vars[0:6])

        #the first index is deticated to the wheel jacobian and the last is for the driving var
        link_jacobians = jacobians[1:]
        j = 0 #link angle index
        for i, link in enumerate(self.linkages):
            num_link_dof = 2 #TODO make this dependent on the type of link involved
            link_jacobians[i] = link.jacobian(link_angles[j:j+num_link_dof])
            j+=num_link_dof


        #creates a jacobian matrix with the derivative of the driving term =0
        jacobian_matrix = block_diag(*jacobians)

        driving_var_vector = np.zeros(self.a_mat[0].shape)
        driving_var_vector[driving_var] = 1

        return np.dot(np.vstack([self.a_mat, driving_var_vector]), jacobian_matrix.T)


    
    def render(self):
        pass


class Five_Link(Kinematic_Model):
    def __init__(self, frame_pickups, link_lengths, upright_pickups):
        linkages = np.zeros(5, dtype=Linkage)

        for i, link in enumerate(linkages):
            linkages[i] = Single_Link(frame_pickups[i], link_lengths[i])

        upright = Upright(upright_pickups)

        super(Five_Link, self).__init__(linkages, upright)


    

        
    