from abc import ABC, abstractmethod
import numpy as np

class Linkage(ABC):
    @abstractmethod
    def position_unit_vector(self):
        pass
    
    @abstractmethod
    def scale_matrix(self):
        pass

#these methods might not be it, more work to be done
class Wheel_Carrier(ABC):
    @abstractmethod
    def rotation_matrix(self):
        pass
    
    @abstractmethod
    def wheel_position_matrix(self):
        pass

#wright tests for these methods
class Single_Link(Linkage):
    def __init__(self, frame_pickup, length):
        self.frame_pickup = frame_pickup
        self.length = length
    
    #override
    def position_unit_vector(self, alpha, beta):
        return np.array([np.cos(beta)*np.sin(alpha), np.cos(beta)*np.sin(alpha)])   
    
    #override
    def scale_matrix(self):
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
    def scale_matrix(self):
        return self.ball_joint_distance * np.identity(3)


class H_Arm:...

class Strut:...

class Trailing_Arm:...

