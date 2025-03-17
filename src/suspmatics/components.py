from abc import ABC, abstractmethod
from typing import override
import typing
import numpy as np
from scipy.linalg import block_diag# type: ignore
from scipy.spatial.transform import Rotation as R# type: ignore
import vpython# type: ignore
from collections.abc import Iterable
import numpy.typing as npt

#custom types
numeric = typing.Union[int, float]
array32 = npt.NDArray[np.float32]

"""
    This matrix transforms from the SAE coordinate system to the vpython coordinate system

    (x_vp = vp_transf_mat * x_sae)

    SAE:
    x -> forward
    y -> starboard (right)
    z -> down

    vpython: TODO check ur notes stoopidface
    x ->
    y ->
    z ->
"""
vp_transf_mat = np.array([[0,-1,0], [0,0,-1], [1,0,0]]) 



class Component(ABC):
    @property
    @abstractmethod
    def input_count(self) -> int: 
        pass

    @property
    @abstractmethod
    def linear_input_count(self) -> int:
        pass

    @property
    @abstractmethod
    def output_count(self) -> int:
        pass

    @property
    @abstractmethod
    def input_names(self) -> list[str]:
        pass

    def __init__(self, datum_vars: array32):
        self.datum_vars = datum_vars


    @abstractmethod
    def local_A_matrix(self) -> array32:
        pass

    @abstractmethod
    def nonlin_x_expression(self, vars: array32) -> array32:
        pass

    @abstractmethod
    def jacobian(self, vars: array32) -> array32:
        pass

    @abstractmethod
    def create_vp_object(self) -> vpython.compound:
        pass

    @abstractmethod
    def update_vp_position(self, vp_object: vpython.compound, vars: array32) -> vpython.compound:
        pass

class Linkage(Component):   
    def __init__(self, datum_vars: array32):
        super().__init__(datum_vars)

    @abstractmethod
    def local_B_vector(self) -> array32:
        pass
class Wheel_Carrier(Component): 
    def __init__(self, datum_vars: array32):
        super().__init__(datum_vars)

    pass


"""
    Single_Link models a rigid tension/compression link with unbounded ball joints at either end 
"""
class Single_Link(Linkage):
    input_count = 2
    linear_input_count = 3
    output_count = 3
    input_names = ["alpha", "beta"]

    def __init__(self, frame_pickup: array32, length: numeric, datum_vars: array32=np.array([np.pi/2, 0])):
        super().__init__(datum_vars)
        self.frame_pickup: array32 = frame_pickup
        self.length = length

    @override
    def local_A_matrix(self) -> array32:
        return -1.0*self.length*np.identity(3)
    
    @override
    def local_B_vector(self) -> array32:
        return self.frame_pickup
    
    @override
    def nonlin_x_expression(self, vars: array32) -> array32:
        alpha, beta = vars
        return np.array([np.cos(beta)*np.cos(alpha), np.cos(beta)*np.sin(alpha), np.sin(beta)])   
    
    @override
    def jacobian(self, vars: array32) -> array32:
        alpha, beta = vars
        #returns [[dx/d_alpha],[dx/d_beta]]
        return np.array([[-np.cos(beta)*np.sin(alpha), np.cos(beta)*np.cos(alpha), 0], [-np.sin(beta)*np.cos(alpha), -np.sin(beta)*np.sin(alpha), np.cos(beta)]])   
    
    @override
    def create_vp_object(self, diameter: numeric=25, color: vpython.color = vpython.color.purple) -> vpython.compound:
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
    def update_vp_position(self, vp_object: vpython.compound, vars: array32) -> vpython.compound:
        vp_object.axis = vpython.vector(*np.dot(vp_transf_mat, self.nonlin_x_expression(vars)))
        return vp_object
    
"""
    A_Arm models a rigid linkage with a pivot axis (two ball joints) on the inboard side and a ball joint on the outboard side
"""
class A_Arm(Linkage):
    input_count = 1
    linear_input_count = 3
    output_count = 3
    input_names = ["alpha"]
    
    def __init__(self, frame_pickup_0: array32, frame_pickup_1: array32, ball_joint_pos: array32):
        self.frame_pickup_0: array32 = frame_pickup_0
        self.frame_pickup_1: array32 = frame_pickup_1

        self.ball_joint_pos: array32 = ball_joint_pos

        pickup_1_to_0: array32 = frame_pickup_0-frame_pickup_1
        rotation_axis_unit_vector: array32 = np.atleast_2d(pickup_1_to_0 / np.linalg.norm(pickup_1_to_0))

        #saves pivot axis as unit vector
        pivot: array32 = frame_pickup_0 - frame_pickup_1
        self.pivot_axis: array32 = pivot / np.linalg.norm(pivot)

        #https://en.wikipedia.org/wiki/Rotation_matrix
        self.cross_product_matrix: array32 = np.array([[                            0, -rotation_axis_unit_vector[0,2], rotation_axis_unit_vector[0,1] ],
                                              [ rotation_axis_unit_vector[0,2],                             0, -rotation_axis_unit_vector[0,0]],
                                              [-rotation_axis_unit_vector[0,1],  rotation_axis_unit_vector[0,0],                             0]])
        self.outer_product_matrix: array32 = np.dot(rotation_axis_unit_vector.T, rotation_axis_unit_vector)

        #this is the point on the pivot axis where the ball joint position vector is orthogonal to axis
        self.orthogonal_link_position: array32 = np.dot(self.outer_product_matrix, np.atleast_2d(self.ball_joint_pos).T)

    @override
    def local_A_matrix(self) -> array32:
        ball_joint_column_vec = np.atleast_2d(self.ball_joint_pos).T

        cos_column = np.dot((np.identity(3)-self.outer_product_matrix), ball_joint_column_vec)
        sin_column = np.dot(self.cross_product_matrix, ball_joint_column_vec)

        return np.block([cos_column, sin_column])
    
    @override
    def local_B_vector(self) -> array32:
        return self.frame_pickup_0 - self.orthogonal_link_position

    @override
    def nonlin_x_expression(self, vars: array32) -> array32:
        alpha = vars
        return np.array([np.cos(alpha), np.sin(alpha)])
    
    @override
    def jacobian(self, vars: array32) -> array32:
        alpha = vars
        return np.array([-np.sin(alpha), np.cos(alpha)]) 
    
    #TODO implement
    @override
    def create_vp_object(self) -> vpython.compound:
        ...
    #TODO implement
    @override
    def update_vp_position(self, vp_object: vpython.compound, vars: array32) -> vpython.compound:
        ...

        
#TODO implement
class H_Arm:...
class Strut:...
class Trailing_Arm:...


"""
    Upright can be used to model a wheel carrier bounded by a collection of ball joints
"""
class Upright(Wheel_Carrier):
    input_count: int = 6
    linear_input_count: int = 12
    input_names: list[str] = ["x", "y", "z", "theta", "phi", "gamma"]
    #output_count = 15#TODO update this BS
    
    def __init__(self, pickups: array32, datum_vars: array32=np.array([0.,0.,0.,0.,0.,0.])):
        super().__init__(datum_vars)
        self.pickup_count: int = pickups.shape[0]
        self.pickups = pickups

    @property
    @override
    def output_count(self) -> int:
        return 3*self.pickup_count

    @override
    def local_A_matrix(self) -> array32:
        #TODO A_pickups generation code can definitely be improved
        A_wheel = np.block([[np.identity(3)]]*self.pickup_count)

        A_pickups: array32 = np.zeros(shape=self.pickups.shape, dtype= np.ndarray)

        for i, pickup in enumerate(self.pickups):
            for j, pickup_coord in enumerate(pickup):
                A_pickups[i,j] = pickup_coord*np.identity(3)

        A_pickups = np.block(A_pickups.tolist())

        return np.block([A_wheel, A_pickups])
    
    @override
    def nonlin_x_expression(self, vars: array32) -> array32:
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars
         
        return np.concatenate([[wheel_x, wheel_y, wheel_z], R.from_euler('XYZ', [theta, phi, gamma]).as_matrix().T.flatten()])
    
    @override
    def jacobian(self, vars: array32) -> array32:
        wheel_x, wheel_y, wheel_z, theta, phi, gamma = vars

        wheel_jac = np.identity(3)

        #TODO this whole thing can be made so much more efficient
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
        
        return array32(block_diag(wheel_jac, [dr_dtheta, dr_dphi, dr_dgamma]))

    @override
    def create_vp_object(self, diameter: numeric=15, axis_len: numeric=40, color: vpython.color=vpython.color.cyan) -> vpython.compound:
        output = np.zeros(3+2*self.pickup_count, dtype=vpython.standardAttributes)
        r = diameter/2

        #the first 3 objects are the basis vectors for the local frame of reference
        output[0] = vpython.arrow(axis=vpython.vector(0,0,axis_len), color=vpython.color.blue)
        output[1] = vpython.arrow(axis=vpython.vector(-axis_len,0,0), color=vpython.color.green)#wheel is axisymmetric so these mfs prolly aren't necessary
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
    def update_vp_position(self, vp_object: vpython.compound, vars: array32) -> vpython.compound:
        theta, phi, gamma = vars[3:]

        #r = np.dot(vp_transf_mat, R.from_euler('XYZ', [theta, phi, gamma]).as_matrix())
        r = vp_transf_mat.dot(R.from_euler('XYZ', [theta, phi, gamma]).as_matrix())


        axis = vpython.vector(*r.dot(np.array([0,-1,0])))
        up = vpython.vector(*r.dot(np.array([0,0,-1])))

        vp_object.axis = axis
        vp_object.up = up
        vp_object.pos = vpython.vector(*np.dot(vp_transf_mat, vars[:3]))

        return vp_object



