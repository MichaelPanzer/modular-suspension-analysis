import numpy as np
from suspmatics.components import *
from scipy.linalg import block_diag
from scipy.linalg import lu
from collections.abc import Iterable

class Kinematic_Model:
 
    def __init__(self, linkages: Iterable, wheel_carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier

        self.components = np.concatenate(([self.wheel_carrier], self.linkages))

        self.a_mat = self.global_A_matrix()
        self.b_vec = self.global_B_vector()
        #self.linear_system, self.frame_pickups = self.__generate_linear_system__()

    @classmethod
    def from_file(self, file_name):
        pass

    @classmethod
    def five_link(self, frame_pickups, link_lengths, upright_pickups):
        linkages = np.zeros(5, dtype=Linkage)

        for i, (pickup, length) in enumerate(zip(frame_pickups, link_lengths)):
            linkages[i] = Single_Link(pickup, length)

        upright = Upright(upright_pickups)

        return Kinematic_Model(linkages, upright)
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
        x = np.zeros(27) # fix this so inital length is the sum of all the linear input counts


        x[0:self.wheel_carrier.linear_input_count] = self.wheel_carrier.nonlin_x_expression(vars[0:self.wheel_carrier.input_count]) #wheel carrier position and rotation

        link_vecs = x[self.wheel_carrier.linear_input_count:]
        link_angles = vars[self.wheel_carrier.input_count:]

        i = 0 #link vec index
        j = 0 #link angle index
        for linkage in self.linkages:                 
            link_vecs[i:i+linkage.output_count] = linkage.nonlin_x_expression(link_angles[j:j+linkage.input_count])

            i += linkage.output_count
            j += linkage.input_count

        return x
    
    #TODO delete this
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
    
    def full_sys_of_eq(self, vars, driving_vals):#driving vals is a tuple of lists ([indexes], [values])
        x = self.generate_x_nonlinear(vars)

        nonlin_expressions = (np.dot(self.a_mat, x).T - self.b_vec.T)[0]
        
        #if not isinstance(driving_vals, Iterable):
            #driving_vals = [driving_vals, ]

        driving_exprs = np.zeros(len(driving_vals[0]))
        for i, (var_index, value)  in enumerate(zip(driving_vals[0], driving_vals[1])):
            driving_exprs[i] = x[var_index] - value


        return np.concatenate((nonlin_expressions, driving_exprs))
    
    #this probably isnt nessecary
    def approx_sys_of_eq(self, vars, driving_vals):
            x = self.approx_x_nonlinear(vars)

            nonlin_expressions = (np.dot(self.a_mat, x).T - self.b_vec.T)[0]

            #print(nonlin_expressions)

            driving_exprs = np.zeros(len(driving_vals[0]))
            for i, (var_index, value)  in enumerate(zip(driving_vals[0], driving_vals[1])):
                driving_exprs[i] = x[var_index] - value

            return np.concatenate((nonlin_expressions, np.array([driving_exprs])))
    
    #TODO write some fucking tests stoopidhead
    def jacobian(self, vars, driving_var_indices):
        jacobians = np.zeros(self.linkages.shape[0] + 1, dtype=np.ndarray)
        link_angles = vars[self.wheel_carrier.input_count:]
        jacobians[0] = self.wheel_carrier.jacobian(vars[0:self.wheel_carrier.input_count])

        #the first index is deticated to the wheel jacobian
        link_jacobians = jacobians[1:]
        j = 0 #link input index
        for i, link in enumerate(self.linkages):
            link_jacobians[i] = link.jacobian(link_angles[j:j+link.input_count])
            j+=link.input_count


        #creates a jacobian matrix with the derivative of the driving terms =0
        jacobian_matrix = block_diag(*jacobians)    

        driving_var_matrix = np.zeros((len(driving_var_indices),) + self.a_mat[0].shape)
        for i, var_index in enumerate(driving_var_indices):
            driving_var_matrix[i,var_index] = 1

        return np.dot(np.vstack([self.a_mat, driving_var_matrix]), jacobian_matrix.T)

    def render(self, vars):
        self.wheel_carrier.update_vp_position(vars[0:6])

        link_vars = vars[6:]
        i=0
        for link in self.linkages:
            link_var_count = 2
            link.update_vp_position(link_vars[i:i+link_var_count])
            i+=link_var_count
        
        return


class Five_Link(Kinematic_Model):
    def __init__(self, frame_pickups, link_lengths, upright_pickups):
        linkages = np.zeros(5, dtype=Linkage)

        for i, link in enumerate(linkages):
            linkages[i] = Single_Link(frame_pickups[i], link_lengths[i])

        upright = Upright(upright_pickups)

        super(Five_Link, self).__init__(linkages, upright)


    

        
    