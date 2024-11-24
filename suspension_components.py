from abc import ABC, abstractmethod
import numpy as np

class Linkage(ABC):
    @abstractmethod
    def local_A_matrix(self):
        pass

    @abstractmethod
    def nonlin_x_expression(self, vars):
        pass

    @abstractmethod
    def render(self, vars):
        pass

#these methods might not be it, more work to be done
class Wheel_Carrier(ABC):
    @abstractmethod
    def rotation_matrix(self):
        pass
    
    @abstractmethod
    def wheel_position_matrix(self):
        pass

#right tests for these methods
class Single_Link(Linkage):
    def __init__(self, frame_pickup, length):
        self.frame_pickup = frame_pickup
        self.length = length
    
    #override
    def local_A_matrix(self):
        return self.length*np.identity(3)
    
    #override
    def nonlin_x_expression(self, vars):
        alpha, beta = vars
        return np.array([np.cos(beta)*np.cos(alpha), np.cos(beta)*np.sin(alpha), np.sin(beta)])   
    
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

    #override
    def local_A_matrix(self):
        ball_joint_colum_vec = np.atleast_2d(self.ball_joint_pos).T

        cos_colum = np.dot((np.identity(3)-self.outer_product_matrix), ball_joint_colum_vec)
        sin_colum = np.dot(self.cross_product_matrix, ball_joint_colum_vec)

        return np.block([cos_colum, sin_colum]), np.dot(self.outer_product_matrix, np.atleast_2d(self.ball_joint_pos).T)
    
    #override
    def nonlin_x_expression(self, vars):
        alpha = vars

        return np.array([np.cos(alpha), np.sin(alpha)])
    
    #override TODO
    def render(self):
        ...

#TODO add stuff for these 
class H_Arm:...

class Strut:...

class Trailing_Arm:...

#add methods here 
class Upright(Wheel_Carrier):
    def __init__(self, pickups):
        self.pickups = pickups
        self.pickup_count = pickups.len()
    
    #override
    def rotation_matrix(theta, phi, gamma):
        ... #TODO copy from other thingy
    
    #override
    def wheel_position_matrix(self):
        A_pickup = np.dot(np.identity, self.pickups)
        A_wheel = np.array([np.identity*self.pickup_count])
        return A_wheel, A_pickup
