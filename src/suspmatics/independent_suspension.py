import numpy as np
import numpy.typing as npt
import typing
import numpy.typing as npt
import scipy as sp# type: ignore
from suspmatics.components import * #TODO remove this import and reference with components.foo
from suspmatics import components
from scipy.linalg import block_diag# type: ignore
import itertools
from collections import abc

#numeric = typing.Union[int, float]
#a32 = npt.NDArray[np.float32]

#returns index of nearest(or possibly 2nd nearest) value
def modified_interpolation_search(value: float, sorted_list: array32) -> int:
    low: int = 0
    high: int = len(sorted_list) - 1
    pos: int = -1
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

    return pos

class Kinematic_Model:
    def __init__(self, linkages: list[Linkage], wheel_carrier: Wheel_Carrier):
        self.linkages = linkages
        self.wheel_carrier = wheel_carrier

        #self.components = np.array([self.wheel_carrier: Component] + self.linkages: Iterable[Component])
        self.components: list[Component] = [self.wheel_carrier] + list[Component](self.linkages)

        #this list stores the final var index of each component. To access the vars that correspond to a specific components use [input__indices[comp_num]-comp.input_count : input__indices[comp_num]]
        comp_input_counts = np.array([c.input_count for c in self.components])
        end_indices = np.array(list(itertools.accumulate(comp_input_counts)))
        self.input_indices: list[tuple[int, int]] = list(zip(end_indices-comp_input_counts, end_indices))

        #stores callable functions for the nonlinear terms and jacobians of each component
        self.nonlinear_functions: list[abc.Callable[[array32], array32]] = [comp.nonlin_expression() for comp in self.components]
        self.jacobian_functions: list[abc.Callable[[array32], array32]] = [comp.jacobian() for comp in self.components]


        self.coef_mat = self._global_coef_mat()
        self.fixed_vec = self._global_fixed_vec()
        
    @classmethod
    def five_link(cls, frame_pickups: list[array32], link_lengths: list[np.float32], upright_pickups: array32) -> typing.Self:
        linkages: list[Linkage] = [Single_Link(pickup, length) for (pickup, length) in zip(frame_pickups, link_lengths)]
        upright = Upright(upright_pickups)

        return cls(linkages, upright)
    
    #generates the matrix of coefficients to the linearized system of eqs
    def _global_coef_mat(self) -> array32:
        wc_local_coef_mat = self.wheel_carrier.local_coef_mat()
        links_local_coef_mat: list[array32] = [linkage.local_coef_mat() for linkage in self.linkages]

        #TODO refactor with array slices to limit data copying
        links_local_coef_mat = block_diag(*links_local_coef_mat)
        coef_mat = np.block([wc_local_coef_mat, links_local_coef_mat])
        
        return coef_mat
    
    #generates the solution to the linearized system of eqs
    def _global_fixed_vec(self) -> array32:
        fixed_vec = [linkage.local_fixed_vec() for linkage in self.linkages]
        
        #see if its possible to get rid of transpose
        return np.atleast_2d(np.block(fixed_vec)).T

    #generates the input to the linear system of eqs using the non-linear input(actual spatial pos/angles of the components)
    def _nonlin_vec(self, vars: array32) -> array32:
        """
        x: array32 = np.zeros(27, dtype=np.float32) # fix this so initial length is the sum of all the linear input counts

        x[0:self.wheel_carrier.linear_input_count] = self.wheel_carrier.nonlin_x_expression(vars[0:self.wheel_carrier.input_count]) #wheel carrier position and rotation

        link_vecs = x[self.wheel_carrier.linear_input_count:]
        link_angles = vars[self.wheel_carrier.input_count:]

        i = 0 #link vec index
        j = 0 #link angle index
        for linkage in self.linkages:                 
            link_vecs[i:i+linkage.output_count] = linkage.nonlin_x_expression(link_angles[j:j+linkage.input_count])

            i += linkage.output_count
            j += linkage.input_count
        """
        x = np.block([f(vars[index1:index2]) for (f, (index1, index2)) in zip(self.nonlinear_functions, self.input_indices)])

        return x
    
    
    def full_sys_of_eq(self, vars: array32, driving_vals: list[tuple[int, components.numeric]]) -> array32:
        """
        Returns the full system of equations generated by the kinematic model

        The resulting vector represents the error caused by a specific input.
        When the output of this method is 0, the system is solved

        driving vals is a list of tuples [(index, value)] that are known beforehand; more than one driving val over-constrains the system
        """
        x = self._nonlin_vec(vars)
        nonlin_expressions = (np.dot(self.coef_mat, x).T - self.fixed_vec.T)[0]
        driving_exprs = np.array([x[var_index]-value   for(var_index, value)  in driving_vals])

        return np.concatenate((nonlin_expressions, driving_exprs))
    
    def jacobian(self, vars: array32, driving_vals: list[tuple[int, components.numeric]]) -> array32:
        
        jacobians: list[array32] = [f(vars[index1:index2]) for (f, (index1, index2)) in zip(self.jacobian_functions, self.input_indices)]
        print(self.input_indices)
        
        #TODO test and delete
        """
        jacobians = np.zeros(self.linkages.shape[0] + 1, dtype=np.ndarray)
        link_angles = vars[self.wheel_carrier.input_count:]
        jacobians[0] = self.wheel_carrier.jacobian(vars[0:self.wheel_carrier.input_count])

        #the first index is dedicated to the wheel jacobian
        link_jacobians = jacobians[1:]
        j = 0 #link input index
        for i, link in enumerate(self.linkages):
            link_jacobians[i] = link.jacobian(link_angles[j:j+link.input_count])
            j+=link.input_count
        """

        #creates a jacobian matrix with the derivative of the driving terms =0
        jacobian_matrix: array32 = block_diag(*jacobians) 


        #TODO find a way to remove the for loop from this
        driving_var_matrix: array32 = np.zeros((len(driving_vals),) + self.coef_mat[0].shape, dtype=np.float32)
        for i, val in enumerate(driving_vals):
            driving_var_matrix[i,val[0]] = 1
        
        #TODO figure out how to make typing work without output variable
        output: array32 = np.dot(np.vstack([self.coef_mat, driving_var_matrix]), jacobian_matrix.T)
        return output

    def initial_guess(self, driving_vals: list[tuple[int, components.numeric]], conv_tol: float = 0.1) -> sp.optimize.OptimizeResult:
        """
        Initial guess creates an approximate solution to an over-constrained kinematic model

        Constraints should be chosen to generate the correct orientation of the components
        """
        #function representing the system of equations used in the least squares approximation
        def function(vars: array32) -> tuple[array32, array32]:
            return self.full_sys_of_eq(vars, driving_vals), self.jacobian(vars, driving_vals)


        guess: array32 = np.concatenate([comp.init_vars for comp in self.components])
        for (index, value) in driving_vals:
            guess[index] = value

        return sp.optimize.root(function, guess, jac=True, method="lm")

    #TODO write test for this
    #TODO change return type to scipy.spatial.cKDTree
    def solution_mat(self, initial_values: list[tuple[int, numeric]], driving_var_index: int, driving_var_range: array32) -> array32:
        #Init guess to start with links in correct orientation
        guess = self.initial_guess(initial_values).x

        #Function to for a single value of the driving variable 
        def solve_f_var(driving_value: numeric, guess: array32) -> array32:
            #function representing the system of equations used in numerical solver
            def function(vars: array32) -> tuple[array32, array32]:
                driving_var = [(driving_var_index, driving_value)]
                return self.full_sys_of_eq(vars, driving_var), self.jacobian(vars, driving_var)
            #TODO this is dumb
            output: array32 = sp.optimize.root(function, guess, jac=True, method = 'hybr').x 
            return output

        #Finds initial solution closest to initial guess
        initial_solution = solve_f_var(guess[driving_var_index], guess)
        starting_index = modified_interpolation_search(initial_solution[driving_var_index], driving_var_range)

        solutions: array32 = np.zeros([len(driving_var_range),len(initial_solution)], dtype=np.float32)

        #Solves everything above starting index
        upper_driving_var_range = driving_var_range[starting_index:]
        upper_solutions = solutions[starting_index:]
        guess = initial_solution
        for i, driving_value in enumerate(upper_driving_var_range):
            upper_solutions[i] = solve_f_var(driving_value, guess)
            guess = upper_solutions[i]

        #Solves everything below starting index
        lower_driving_var_range = driving_var_range[:starting_index][::-1]
        lower_solutions = solutions[:starting_index][::-1]
        guess = initial_solution
        for i, driving_value in enumerate(lower_driving_var_range):
            lower_solutions[i] = solve_f_var(driving_value, guess)
            guess = lower_solutions[i]

        return solutions




        
    