import numpy as np
import math
from quanterion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np


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

    # Get global_frame T marker frame 
    marker_in_global_frame = np.eye(4)
    marker_in_global_frame[0:3, 0:3] = marker_rot
    marker_in_global_frame[0:3, 3:4] = marker_pos


    quanterion = Quaternion(obj.pose.q0, obj.pose.q1, obj.pose.q2, obj.pose.q3) 
    R_cam = quaternion_rotation_matrix(quanterion)
    t_cam = np.array([[obj.pose.x],
                    [obj.pose.y],
                    [obj.pose.z]])
    
    marker_in_camera_frame = np.eye(4)
    marker_in_camera_frame[0:3, 0:3] = R_cam
    marker_in_camera_frame[0:3, 3:4] = t_cam

    # Step 1: Marker in robot frame
    marker_in_robot = np.linalg.inv(marker_in_camera_frame)

    # print(f' MARKER IN ROBOT -> {marker_in_robot[0:3, 3:4]}\n')

    # Step 3: Get robot in global frame
    robot_in_global = marker_in_global_frame @ marker_in_robot

    pos_world = robot_in_global[0:3, 3:4]


    # print(f'CALCULATED POSE {pos_world}\n')

    return pos_world



#CHATGPT
def angle_mean(angle1, angle2, alpha):
    # Convert angles to unit vectors
    x = alpha * np.cos(angle1) + (1 - alpha) * np.cos(angle2)
    y = alpha * np.sin(angle1) + (1 - alpha) * np.sin(angle2)
    return np.arctan2(y, x)

# Scaling function 
def scale_factor(raw, marker_name):

    y = 0
    if marker_name == "Circle":
        y = -5.8458 * raw + 1590.7008
    elif marker_name == "Diamond":
        y = 6.6093 * raw - 1986.9060
    elif marker_name == "Hexagon":
        y = 5.6176 * raw + 1654.8487

# Model: true = -3.8178 * raw + 1058.5996
# Model: true = 6.1470 * raw + -1829.8999
# Model: true = 5.2695 * raw + 1607.5901

    return y