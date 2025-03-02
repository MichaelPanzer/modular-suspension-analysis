import numpy as np
import scipy as sp
from suspmatics.components import *
from scipy.linalg import block_diag
from scipy.linalg import lu
from collections.abc import Iterable

#figure out how to give value a generic type
def modified_interpolation_search(value, sorted_list: Iterable) -> int:
    low = 0
    high = len(sorted_list) - 1
    while low <= high and value >= sorted_list[low] and value <= sorted_list[high]:
        pos = int(low + ((high - low) // (sorted_list[high] - sorted_list[low])) * (value - sorted_list[low])) #Linear interpolation to find approximate pos
        #checks to see if value is just below pos
        if value < sorted_list[pos]:
            if value > sorted_list[pos-1]:
                return pos
            high = pos - 1
        
        #checks to see if value is just above pos
        elif value > sorted_list[pos]:
            if value < sorted_list[pos+1]:
                return pos
            low = pos + 1
        
        #only true if value=sorted_list[pos]
        else:
            return pos
    
class Kinematic_Model:
 
    def __init__(self, linkages: Iterable[Linkage], wheel_carrier: Wheel_Carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier

        #self.components = np.array([self.wheel_carrier: Component] + self.linkages: Iterable[Component])
        self.components = np.concatenate(([self.wheel_carrier], self.linkages))
        self.input_count = sum(comp.input_count for comp in self.components)


        self.a_mat = self._global_A_matrix()
        self.b_vec = self._global_B_vector()
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
    
    def _global_A_matrix(self):
        wheelcarrier_local_A = self.wheel_carrier.local_A_matrix()
        link_local_A = np.zeros(self.linkages.size, dtype=np.ndarray)

        for i, linkage in enumerate(self.linkages):
            link_local_A[i] = linkage.local_A_matrix()

        A_links = block_diag(*link_local_A)


        A_matrix = np.block([wheelcarrier_local_A, A_links])
        
        
        return A_matrix
    
    def _global_B_vector(self):
        B_vector = np.zeros(self.linkages.size, dtype=np.ndarray)

        for i, linkage in enumerate(self.linkages):
            B_vector[i] = linkage.local_B_vector()
        
        return np.atleast_2d(np.block(B_vector.tolist())).T

    def _generate_x_nonlinear(self, vars):
        x = np.zeros(27) # fix this so initial length is the sum of all the linear input counts


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
    
    def full_sys_of_eq(self, vars, driving_vals: Iterable):
        """
        driving vals is a tuple of lists ([indexes], [values])
        """
        x = self._generate_x_nonlinear(vars)

        nonlin_expressions = (np.dot(self.a_mat, x).T - self.b_vec.T)[0]
        
        #if not isinstance(driving_vals, Iterable):
            #driving_vals = [driving_vals, ]

        driving_exprs = np.zeros(len(driving_vals[0]))
        for i, (var_index, value)  in enumerate(zip(driving_vals[0], driving_vals[1])):
            driving_exprs[i] = x[var_index] - value


        return np.concatenate((nonlin_expressions, driving_exprs))
    
    def jacobian(self, vars, driving_var_indices):
        jacobians = np.zeros(self.linkages.shape[0] + 1, dtype=np.ndarray)
        link_angles = vars[self.wheel_carrier.input_count:]
        jacobians[0] = self.wheel_carrier.jacobian(vars[0:self.wheel_carrier.input_count])

        #the first index is dedicated to the wheel jacobian
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

    def fixed_sys_of_eq(self, vars, fixed_vals):
        """
        driving vals is an iterable of tuples [(indexes, value)]
        This creates a system of equations with the driving vals fixed and vars representing all non driven parameters
        """

        full_vars = np.zeros(len(vars)+len(fixed_vals))

        driving_vals_iter = iter(fixed_vals)
        vars_iter = iter(vars)

        current_driving_val = next(driving_vals_iter)

        for var, i in enumerate(full_vars):
            #This checks if it is at the index of a driving val
            if(current_driving_val[0] == i):
                #At driving val index, insert value
                full_vars[i] = current_driving_val[1]
                current_driving_val = next(driving_vals_iter)
            else:
                #At anything other than driving val index, insert var
                full_vars[i] = next(vars_iter)

    def initial_guess(self, driving_vals: tuple[Iterable[int], Iterable[Number]], conv_tol = 0.1):#fixed vals is a tuple of lists ([indexes], [values]) 
       
        def function(vars):
            return self.full_sys_of_eq(vars, driving_vals)
    
        def jacobian(vars):
            return self.jacobian(vars, driving_vals[0])

        #find guess based on default position of links & fixed
        #guess = np.zeros(self.input_count + len(fixed_vals)) #make length total number of inputs to the function + length of fixed_vals
        #fill guess with default link positions
        guess = np.array(np.concatenate([comp.datum_vars for comp in self.components]))

        #print(guess)

        return sp.optimize.root(function, guess, jac=jacobian, method="lm")

    #TODO write test for this
    #This finds the index of the item of the list closest to the value

    def create_table(self, initial_values: tuple[Iterable[int], Iterable[Number]], driving_var_index: int, driving_var_range: Iterable[Number]) -> np.ndarray:
        guess = self.initial_guess(initial_values).x

        def solve_f_var(driving_value, guess):
            def function(vars):
                return self.full_sys_of_eq(vars, ([driving_var_index], [driving_value]))
            
            def jacobian(vars):
                return self.jacobian(vars, [driving_var_index])
            
            return sp.optimize.root(function, guess, jac=jacobian, method = 'hybr').x

        initial_solution = solve_f_var(guess[driving_var_index], guess)
        starting_index = modified_interpolation_search(initial_solution[driving_var_index], driving_var_range)

        solutions = np.zeros([len(driving_var_range),len(initial_solution)])

        #solves everything above starting index
        upper_driving_var_range = driving_var_range[starting_index:]
        upper_solutions = solutions[starting_index:]
        guess = initial_solution
        for i, driving_value in enumerate(upper_driving_var_range):
            upper_solutions[i] = solve_f_var(driving_value, guess)
            guess = upper_solutions[i]

        #solves everything below starting index
        lower_driving_var_range = driving_var_range[:starting_index][::-1]
        lower_solutions = solutions[:starting_index][::-1]
        guess = initial_solution
        for i, driving_value in enumerate(lower_driving_var_range):
            lower_solutions[i] = solve_f_var(driving_value, guess)
            guess = lower_solutions[i]

        return solutions






        
        



    def render(self, vars):
        self.wheel_carrier.update_vp_position(vars[0:6])

        link_vars = vars[6:]
        i=0
        for link in self.linkages:
            link_var_count = 2
            link.update_vp_position(link_vars[i:i+link_var_count])
            i+=link_var_count
        
        return


        
    