import numpy as np
import math
from quanterion import Quaternion


def is_vector_moving(robot, velocity_threshold=2.0):
    left_speed = robot.left_wheel_speed_mmps
    right_speed = robot.right_wheel_speed_mmps

    return abs(left_speed) > velocity_threshold or abs(right_speed) > velocity_threshold


### GOTTEN FROM GEEKS FOR GEEKS
def quaternion_rotation_matrix(Q):
    """
    Convert a quaternion into a full three-dimensional rotation matrix.
    """
    q0 = Q.q0
    q1 = Q.q1
    q2 = Q.q2
    q3 = Q.q3

    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ])

def frame_transformation(obj, marker_pos, marker_rot):
    ## Build Homogenous Transofrmation Matrix 
    marker_matrix = np.eye(4)
    marker_matrix[0:3, 0:3] = marker_rot
    marker_matrix[0:3, 3:4] = marker_pos

    quanterion = Quaternion(obj.pose.q0, obj.pose.q1, obj.pose.q2, obj.pose.q3) 
    


    # Detected Marker
    R = quaternion_rotation_matrix(quanterion)
    t = np.array([[obj.pose.x],
                [obj.pose.y],
                [obj.pose.z]])
    

    # print(f'ROTATION--> {R} POSITION --> {t}')
    # Build Homogenous Transformation Matrix of Detection
    detected_matrix = np.eye(4)
    detected_matrix[0:3, 0:3] = R  
    detected_matrix[0:3, 3:4] = t

    # Inverse it
    inv_detected = np.linalg.inv(detected_matrix)

    # Find pose in global coordinate system
    global_pose = marker_matrix @ inv_detected

    pos_world = global_pose[0:3, 3]

    return pos_world

#CHATGPT
def angle_mean(angle1, angle2, alpha):
    # Convert angles to unit vectors
    x = alpha * np.cos(angle1) + (1 - alpha) * np.cos(angle2)
    y = alpha * np.sin(angle1) + (1 - alpha) * np.sin(angle2)
    return np.arctan2(y, x)