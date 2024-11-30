from abc import ABC, abstractmethod
import numpy as np
import logging

sin_approx_a = 24/np.pi**4
cos_approx_a = (60*np.pi**2 - 720) / np.pi**5
cos_approx_b = (60 - 3*np.pi**2) / np.pi**3

def a_sin(angle):
    return sin_approx_a*angle

def a_cos(angle):
    return cos_approx_a*angle**2 + cos_approx_b



#TODO maybe try to find a better class structure because linakge and wheel carrier have the same methods
class Linkage(ABC):
    @abstractmethod
    def local_A_matrix(self):
        pass

    @abstractmethod
    def local_B_vector(self):
        pass

    @abstractmethod
    def nonlin_x_expression(self, vars):
        pass

    @abstractmethod
    def approx_nonlin_x(self, vars):
        pass


    @abstractmethod
    def jacobian(self, vars):
        pass

    @abstractmethod
    def render(self, vars):
        pass

class Wheel_Carrier(ABC):
    @abstractmethod
    def local_A_matrix(self):
        pass

    @abstractmethod
    def nonlin_x_expression(self, vars):
        pass

    @abstractmethod
    def nonlin_x_expression(self, vars):
        pass

    @abstractmethod
    def jacobian(self, vars):
        pass
    @abstractmethod
    def render(self, vars):
        pass

#wright tests for these methods
class Single_Link(Linkage):
    def __init__(self, frame_pickup, length):
        self.frame_pickup = frame_pickup
        self.length = length
    
    #override
    def local_A_matrix(self):
        return -1*self.length*np.identity(3)
    
    #override
    def local_B_vector(self):
        return self.frame_pickup
    
    #override
    def nonlin_x_expression(self, vars):
        alpha, beta = vars
        return np.array([np.cos(beta)*np.cos(alpha), np.cos(beta)*np.sin(alpha), np.sin(beta)])   
    
    #override
    def approx_nonlin_x(self, vars):
        alpha, beta = vars
        return np.array([a_cos(beta)*a_cos(alpha), a_cos(beta)*a_sin(alpha), a_sin(beta)])
    
    #override
    def jacobian(self, vars):
        alpha, beta = vars
        #returns [[dx/d_alpha],[dx/d_beta]]
        return np.array([[-np.cos(beta)*np.sin(alpha), np.cos(beta)*np.cos(alpha), np.sin(beta)], [-np.sin(beta)*np.cos(alpha), -np.sin(beta)*np.sin(alpha), np.cos(beta)]])   
    
    #override TODO
    def render(self):
        ...

class A_Arm(Linkage):
    def __init__(self, frame_pickup_0, frame_pickup_1, ball_joint_pos):
        self.frame_pickup_0 = frame_pickup_0
        self.frame_pickup_1 = frame_pickup_1

        self.ball_joint_pos = ball_joint_pos

        pickup_1_to_0 = frame_pickup_0-frame_pickup_1
        rotation_axis_unit_vector = np.atleast_2d(pickup_1_to_0 / np.linalg.norm(pickup_1_to_0))

        #print(rotation_axis_unit_vector)

        #saves pivot axis as unit vector
        pivot = frame_pickup_0 - frame_pickup_1
        self.pivot_axis = pivot / np.linalg.norm(pivot)

        #https://en.wikipedia.org/wiki/Rotation_matrix
        self.cross_product_matrix = np.array([[                            0, -rotation_axis_unit_vector[0,2], rotation_axis_unit_vector[0,1] ],
                                              [ rotation_axis_unit_vector[0,2],                             0, -rotation_axis_unit_vector[0,0]],
                                              [-rotation_axis_unit_vector[0,1],  rotation_axis_unit_vector[0,0],                             0]])
        self.outer_product_matrix = np.dot(rotation_axis_unit_vector.T, rotation_axis_unit_vector)

        #this is the point on the pivot axis where the ball joint position vector is orthogonal to axis
        self.orthogonal_link_position = np.dot(self.outer_product_matrix, np.atleast_2d(self.ball_joint_pos).T)


    #override
    def local_A_matrix(self):
        ball_joint_colum_vec = np.atleast_2d(self.ball_joint_pos).T

        cos_colum = np.dot((np.identity(3)-self.outer_product_matrix), ball_joint_colum_vec)
        sin_colum = np.dot(self.cross_product_matrix, ball_joint_colum_vec)

        return np.block([cos_colum, sin_colum])
    
    def local_B_vector(self):
        return self.frame_pickup_0 - self.orthogonal_link_position

    #override
    def nonlin_x_expression(self, vars):
        alpha = vars
        return np.array([np.cos(alpha), np.sin(alpha)])
    
    #override
    def approx_nonlin_x(self, vars):
        alpha = vars
        return np.array([a_cos(alpha), a_sin(alpha)])
    
    #override
    def jacobian(self, vars):
        alpha = vars
        return np.array([-np.sin(alpha), np.cos(alpha)]) 

    #override TODO
    def render(self):
        ...

#TODO add stuff for these 
class H_Arm:...

class Strut:...

class Trailing_Arm:...



class Upright(Wheel_Carrier):
    def __init__(self, pickups):
        self.pickups = pickups
        self.pickup_count = pickups.shape[0]
    
    #override
    def local_A_matrix(self):
        #TODO A_pickups generation code can definitely be improved
        A_wheel = np.block([[np.identity(3)]]*self.pickup_count)

        A_pickups = np.zeros(shape=self.pickups.shape, dtype= np.ndarray)

        for i, pickup in enumerate(self.pickups):
            for j, pickup_coord in enumerate(pickup):
                A_pickups[i,j] = pickup_coord*np.identity(3)

        A_pickups = np.block(A_pickups.tolist())

        return np.block([A_wheel, A_pickups])
    
    #override
    def nonlin_x_expression(self, vars):
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars

        #TODO this whole thing can be made so much more efficent
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(theta), -1*np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])
        r_y = np.array([[np.cos(phi), 0, np.sin(phi)],
                        [0, 1, 0],
                        [-1*np.sin(phi), 0, np.cos(phi)]])
        r_z = np.array([[np.cos(gamma), -1*np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])

        r = (r_x.dot(r_y).dot(r_z)).T.flatten()
         
        return np.concatenate([[wheel_x, wheel_y, wheel_z], r.tolist()])
    
    #override
    def approx_nonlin_x(self, vars):
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars

        #TODO this whole thing can be made so much more efficent
        r_x = np.array([[1, 0, 0],
                        [0, a_cos(theta), -1*a_sin(theta)],
                        [0, a_sin(theta), a_cos(theta)]])
        r_y = np.array([[a_cos(phi), 0, a_sin(phi)],
                        [0, 1, 0],
                        [-1*a_sin(phi), 0, a_cos(phi)]])
        r_z = np.array([[a_cos(gamma), -1*a_sin(gamma), 0],
                        [a_sin(gamma), a_cos(gamma), 0],
                        [0, 0, 1]])

        r = (r_x.dot(r_y).dot(r_z)).T.flatten()
         
        return np.concatenate([[wheel_x, wheel_y, wheel_z], r.tolist()])
    
        #override
    def jacobian(self, vars):
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars

        wheel_jac = np.identity(3)

        #TODO this whole thing can be made so much more efficent
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])
        
        r_y = np.array([[np.cos(phi), 0, np.sin(phi)],
                        [0, 1, 0],
                        [-np.sin(phi), 0, np.cos(phi)]])
        
        r_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])
        
        r_x_prime = np.array([[1, 0, 0],
                        [0, -np.sin(theta), -np.cos(theta)],
                        [0, np.cos(theta), -np.sin(theta)]])
        
        r_y_prime = np.array([[-np.sin(phi), 0, np.cos(phi)],
                        [0, 1, 0],
                        [-np.cos(phi), 0, np.sin(phi)]])
        
        r_z_prime = np.array([[-np.sin(gamma), -np.cos(gamma), 0],
                        [np.cos(gamma), -np.sin(gamma), 0],
                        [0, 0, 1]])

        dr_dtheta = (r_x_prime.dot(r_y).dot(r_z)).T.flatten()
        dr_dphi = (r_x.dot(r_y_prime).dot(r_z)).T.flatten()
        dr_dgamma = (r_x.dot(r_y).dot(r_z_prime)).T.flatten()
        

        return np.block(np.diag(wheel_jac, [dr_dtheta, dr_dphi, dr_dgamma]))
    
    #override
    def render(self):
        ...#TODO implement this BS