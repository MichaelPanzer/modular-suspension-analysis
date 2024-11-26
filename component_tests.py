import numpy as np
from suspension_components import *
import sympy
import pytest

def test_A_arm_rotation_length_dosent_change(): 
    rx, ry, rz, p1x, p1y, p1z = 15, 3, 6, 0, -5, 2

    outer_ball_joint = np.array([rx, ry, rz])

    a_arm = A_Arm(np.array([0,0,0]), np.array([p1x, p1y, p1z]), outer_ball_joint)

    rotation_mat, offset_vector = a_arm.local_A_matrix()#TODO change this so rotation mat and offset vector are two different operations

    
    cos, sin = np.cos(2), np.sin(2)
    final_ball_joint_pos = np.dot(rotation_mat, np.array([[cos], [sin]])) + offset_vector

    assert np.sum(outer_ball_joint**2) != pytest.approx(np.sum(final_ball_joint_pos**2))

def test_Single_Link_length_dosent_change():
    len = 16.0
    link = Single_Link(np.array([5,2,8]), len)

    alpha = 0.5
    beta = 4

    local_A_mat = link.local_A_matrix()

    nonlin_vector = link.nonlin_x_expression((alpha,beta))

    final_pos = np.dot(local_A_mat, np.atleast_2d(nonlin_vector).T)

    assert len == pytest.approx(np.sqrt(np.sum(final_pos**2)))

def test_Upright_lengths_dont_change():
    upright_pickups = np.array([[1,2,3]])
    upright = Upright(upright_pickups)

    theta = 2
    phi = 1
    gamma = 1.5

    local_A = upright.local_A_matrix()

    x = upright.nonlin_x_expression((0,0,0, theta, phi, gamma))

    assert np.sum(np.dot(local_A, x)**2) == pytest.approx(np.sum(upright_pickups**2))