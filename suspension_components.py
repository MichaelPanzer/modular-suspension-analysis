from abc import ABC, abstractmethod
import numpy as np

class Linkage(ABC):
    @abstractmethod
    def position_unit_vector(self):
        pass
    
    @abstractmethod
    def local_A_matrix(self):
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
    def position_unit_vector(self, alpha, beta):
        return np.array([np.cos(beta)*np.sin(alpha), np.cos(beta)*np.sin(alpha)])   
    
    #override
    def local_A_matrix(self):
        return self.length*np.identity(3)

class A_Arm(Linkage):
    def __init__(self, frame_pickup_0, frame_pickup_1, ball_joint_pos):
        self.frame_pickup_0 = frame_pickup_0
        self.frame_pickup_1 = frame_pickup_1
        self.ball_joint_pos = ball_joint_pos

        self.ball_joint_distance = np.linalg.norm(self.ball_joint_pos)
        self.ball_joint_unit_vector = self.ball_joint_pos / self.ball_joint_distance

        #saves pivot axis as unit vector
        pivot = frame_pickup_0, frame_pickup_1
        self.pivot_axis = pivot / np.linalg.norm(pivot)

        #https://en.wikipedia.org/wiki/Rotation_matrix
        self.cross_product_matrix = np.array([[                              0,-self.ball_joint_unit_vector[2], self.ball_joint_unit_vector[1]],
                                              [ self.ball_joint_unit_vector[2],                              0, -self.ball_joint_unit_vector[0]],
                                              [-self.ball_joint_unit_vector[1], self.ball_joint_unit_vector[2],                              0]])
        self.outer_product_matrix = np.dot(self.ball_joint_unit_vector, self.ball_joint_unit_vector.T)

    #override
    def position_unit_vector(self, alpha):
        cos = np.cos(alpha)
        return cos*np.identity(3) + np.sin(alpha)*self.cross_product_matrix + (1-cos)*self.outer_product_matrix
    
    
    #override
    def local_A_matrix(self):
        ball_joint_colum_vec = np.atleast_2d(self.ball_joint_pos).T

        cos_vector = np.dot((np.identity(3)-self.outer_product_matrix), ball_joint_colum_vec)
        sin_vector = np.dot(self.cross_product_matrix, ball_joint_colum_vec)

        return np.block([cos_vector, sin_vector]) + np.dot(self.outer_product_matrix, ball_joint_colum_vec)

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
