from abc import ABC, abstractmethod
from typing import override
import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation as R
import vpython
import sympy

sin_approx_a = 24/np.pi**4
cos_approx_a = (60*np.pi**2 - 720) / np.pi**5
cos_approx_b = (60-3*np.pi**2) / np.pi**3
def approx_sin(angle):
    return sin_approx_a*angle
def approx_cos(angle):
    return cos_approx_a*angle**2 + cos_approx_b

vp_transf_mat = np.array([[0,-1,0], [0,0,-1], [1,0,0]]) #this matrix transforms from the SAE coordinate system to the vpython coordinate system
class Component(ABC):
    @property
    @abstractmethod
    def input_count(self):
        pass

    @property
    @abstractmethod
    def linear_input_count(self):
        pass

    @property
    @abstractmethod
    def output_count(self):
        pass

    @property
    @abstractmethod
    def input_names(self):
        pass

    @abstractmethod
    def local_A_matrix(self):
        pass

    @abstractmethod
    def nonlin_x_expression(self, vars):
        pass

    @abstractmethod
    def jacobian(self, vars):
        pass

    @abstractmethod
    def create_vp_object():
        pass

    @abstractmethod
    def update_vp_position():
        pass

class Linkage(Component):    
    @abstractmethod
    def local_B_vector(self):
        pass

class Wheel_Carrier(Component): 
    pass


class Single_Link(Linkage):
    input_count = 2
    linear_input_count = 3
    output_count = 3
    input_names = ["alpha", "beta"]

    def __init__(self, frame_pickup, length, datum_vars=(np.pi/2,0)):
        self.frame_pickup = frame_pickup
        self.length = length
        #self.vp_object = 
        self.datum_vars=datum_vars

    @override
    def local_A_matrix(self):
        return -1*self.length*np.identity(3)
    
    @override
    def local_B_vector(self):
        return self.frame_pickup
    
    @override
    def nonlin_x_expression(self, vars):
        alpha, beta = vars
        return np.array([np.cos(beta)*np.cos(alpha), np.cos(beta)*np.sin(alpha), np.sin(beta)])   
    
    @override
    def approx_nonlin_x(self, vars):
        alpha, beta = vars
        datum_alpha, datum_beta = self.datum_vars
        delta_alpha, delta_beta = vars[0]-datum_alpha, vars[1]-datum_beta


        c_dat_a = np.cos(datum_alpha)
        s_dat_a = np.sin(datum_alpha)
        c_dat_b = np.cos(datum_beta)
        s_dat_b = np.sin(datum_beta)

        #datum is used as an offset to keep approximation domain +-90 deg
        cos_beta = approx_cos(delta_beta)*c_dat_b - approx_sin(delta_beta)*s_dat_b
        return np.array([ cos_beta*(approx_cos(delta_alpha)*c_dat_b - approx_sin(delta_alpha)*s_dat_a), 
                         cos_beta*(approx_cos(delta_alpha)*s_dat_a + approx_sin(delta_alpha)*c_dat_a), 
                         approx_cos(delta_beta)*s_dat_b + approx_sin(delta_beta)*c_dat_b])
    
    @override
    def jacobian(self, vars):
        alpha, beta = vars
        #returns [[dx/d_alpha],[dx/d_beta]]
        return np.array([[-np.cos(beta)*np.sin(alpha), np.cos(beta)*np.cos(alpha), 0], [-np.sin(beta)*np.cos(alpha), -np.sin(beta)*np.sin(alpha), np.cos(beta)]])   
    
    @override
    def create_vp_object(self, diameter=25, color = vpython.color.purple):
        x_axis = np.array([1,0,0])#All objects are generated along the +x axis in the vpython frame of reference

        radius = diameter/2
        
        frame_side_cone  = vpython.cone()
        frame_side_cone.axis = vpython.vector(*(-diameter * x_axis))
        frame_side_cone.pos = -frame_side_cone.axis
        frame_side_cone.radius = radius

        cylinder = vpython.cylinder()
        cylinder.axis = vpython.vector(*((self.length-2*diameter)*x_axis))
        cylinder.pos = -frame_side_cone.axis
        cylinder.radius = radius

        wheel_side_cone = vpython.cone()
        wheel_side_cone.axis = vpython.vector(*(diameter*x_axis))
        wheel_side_cone.pos = cylinder.pos+cylinder.axis
        wheel_side_cone.radius = radius

        return vpython.compound([frame_side_cone, cylinder, wheel_side_cone], origin=vpython.vector(0,0,0), pos=vpython.vector(*np.dot(vp_transf_mat, self.frame_pickup)), color=color)

    @override
    def update_vp_position(self, vp_object, angles):
        vp_object.axis = vpython.vector(*np.dot(vp_transf_mat, self.nonlin_x_expression(angles)))
        return self.vp_object
    
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


    @override
    def local_A_matrix(self):
        ball_joint_colum_vec = np.atleast_2d(self.ball_joint_pos).T

        cos_colum = np.dot((np.identity(3)-self.outer_product_matrix), ball_joint_colum_vec)
        sin_colum = np.dot(self.cross_product_matrix, ball_joint_colum_vec)

        return np.block([cos_colum, sin_colum])
    
    def local_B_vector(self):
        return self.frame_pickup_0 - self.orthogonal_link_position

    @override
    def nonlin_x_expression(self, vars):
        alpha = vars
        return np.array([np.cos(alpha), np.sin(alpha)])
    
    @override
    def approx_nonlin_x(self, vars):
        alpha = vars
        return np.array([approx_cos(alpha), approx_sin(alpha)])
    
    @override
    def jacobian(self, vars):
        alpha = vars
        return np.array([-np.sin(alpha), np.cos(alpha)]) 

#TODO add stuff for these 
class H_Arm:...

class Strut:...

class Trailing_Arm:...

class Upright(Wheel_Carrier):
    input_count = 6
    linear_input_count = 12
    input_names = ["x", "y", "z", "theta", "phi", "gamma"]
    #output_count = 15#TODO update this BS
    def __init__(self, pickups):
        self.pickup_count = pickups.shape[0]
        self.pickups = pickups

    @property
    @override
    def output_count(self):
        return 3*self.pickup_count

    @override
    def local_A_matrix(self):
        #TODO A_pickups generation code can definitely be improved
        A_wheel = np.block([[np.identity(3)]]*self.pickup_count)

        A_pickups = np.zeros(shape=self.pickups.shape, dtype= np.ndarray)

        for i, pickup in enumerate(self.pickups):
            for j, pickup_coord in enumerate(pickup):
                A_pickups[i,j] = pickup_coord*np.identity(3)

        A_pickups = np.block(A_pickups.tolist())

        return np.block([A_wheel, A_pickups])
    
    @override
    def nonlin_x_expression(self, vars):
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars
         
        return np.concatenate([[wheel_x, wheel_y, wheel_z], R.from_euler('XYZ', [theta, phi, gamma]).as_matrix().T.flatten()])
    
    @override
    def approx_nonlin_x(self, vars):
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars

        #TODO this whole thing can be made so much more efficent
        r_x = np.array([[1, 0, 0],
                        [0, approx_cos(theta), -1*approx_sin(theta)],
                        [0, approx_sin(theta), approx_cos(theta)]])
        r_y = np.array([[approx_cos(phi), 0, approx_sin(phi)],
                        [0, 1, 0],
                        [-1*approx_sin(phi), 0, approx_cos(phi)]])
        r_z = np.array([[approx_cos(gamma), -1*approx_sin(gamma), 0],
                        [approx_sin(gamma), approx_cos(gamma), 0],
                        [0, 0, 1]])

        r = (r_x.dot(r_y).dot(r_z)).T.flatten()
         
        return np.concatenate([[wheel_x, wheel_y, wheel_z], r.tolist()])
    
    
    @override
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
        
        r_x_prime = np.array([[0, 0, 0],
                        [0, -np.sin(theta), -np.cos(theta)],
                        [0, np.cos(theta), -np.sin(theta)]])
    
        r_y_prime = np.array([[-np.sin(phi), 0, np.cos(phi)],
                        [0, 0, 0],
                        [-np.cos(phi), 0, -np.sin(phi)]])
        
        r_z_prime = np.array([[-np.sin(gamma), -np.cos(gamma), 0],
                        [np.cos(gamma), -np.sin(gamma), 0],
                        [0, 0, 0]])

        dr_dtheta = (r_x_prime.dot(r_y).dot(r_z)).T.flatten()
        dr_dphi = (r_x.dot(r_y_prime).dot(r_z)).T.flatten()
        dr_dgamma = (r_x.dot(r_y).dot(r_z_prime)).T.flatten()
        

        return block_diag(wheel_jac, [dr_dtheta, dr_dphi, dr_dgamma])

    @override
    def create_vp_object(self, diameter=15, axis_len=40, color=vpython.color.cyan):
        output = np.zeros(3+2*self.pickup_count, dtype=vpython.standardAttributes)
        r = diameter/2

        #the first 3 objects are the basis vectors for the local frame of refrence
        output[0] = vpython.arrow(axis=vpython.vector(0,0,axis_len), color=vpython.color.blue)
        output[1] = vpython.arrow(axis=vpython.vector(-axis_len,0,0), color=vpython.color.green)#wheel is axisymetric so these mfs prolly arent nessecary
        output[2] = vpython.arrow(axis=vpython.vector(0,-axis_len,0), color=vpython.color.red)

        #All remaining objects are for the pickup points
        pickup_objects = output[3:]
        for i, pickup in enumerate(self.pickups):
            pickup = np.dot(vp_transf_mat, pickup)

            unit_axis = vpython.hat(vpython.vector(*pickup))
            cylinder_axis = vpython.vector(*pickup) - diameter*unit_axis

            pickup_objects[2*i] = vpython.cylinder(axis=cylinder_axis, radius=r, color=vpython.color.magenta)
            pickup_objects[2*i+1] = vpython.cone(pos=cylinder_axis, axis=unit_axis*diameter, radius=r, color=vpython.color.magenta)
        
        return vpython.compound(output.tolist(), origin=vpython.vector(0,0,0), axis=vpython.vector(1,0,0), up=vpython.vector(0,1,0), color=color)
    
    
    @override
    def update_vp_position(self, vars):
        theta, phi, gamma = vars[3:]

        #r = np.dot(vp_transf_mat, R.from_euler('XYZ', [theta, phi, gamma]).as_matrix())
        r = vp_transf_mat.dot(R.from_euler('XYZ', [theta, phi, gamma]).as_matrix())


        axis = vpython.vector(*r.dot(np.array([0,-1,0])))
        up = vpython.vector(*r.dot(np.array([0,0,-1])))

        self.vp_object.axis = axis
        self.vp_object.up = up
        self.vp_object.pos = vpython.vector(*np.dot(vp_transf_mat, vars[:3]))

        return self.vp_object



