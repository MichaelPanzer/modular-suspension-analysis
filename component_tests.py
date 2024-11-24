import numpy as np
from suspension_components import A_Arm

def test_A_arm_rotation_generation(): 
    #TODO add symbolic stuff here to make the test easier to fix
    outer_ball_joint = np.array([15.0,0.0,0.0])

    a_arm = A_Arm(np.array([0,0,0]), np.array([5,6,7]), outer_ball_joint)

    rotation_mat = a_arm.local_A_matrix()

    final_ball_joint_pos = np.dot(rotation_mat, np.array([[np.cos(2)], [np.sin(2)]]))

    print(final_ball_joint_pos)
    print(outer_ball_joint)

    assert np.sum(outer_ball_joint**2) == np.sum(final_ball_joint_pos**2)