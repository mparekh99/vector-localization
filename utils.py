import numpy as np
import math
from quanterion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np


def rotation_z(theta):
    # theta = np.radians(degrees_angle)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])


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
    marker_in_global_frame[:3, :3] = marker_rot
    marker_in_global_frame[:3, 3] = marker_pos.flatten()

    quanterion = Quaternion(obj.pose.q0, obj.pose.q1, obj.pose.q2, obj.pose.q3)

    print(obj.pose.q0, obj.pose.q1, obj.pose.q2, obj.pose.q3) 
    R_marker_in_camera = quaternion_rotation_matrix(quanterion)

    t_marker_in_camera = np.array([[obj.pose.x],
                    [obj.pose.y],
                    [obj.pose.z]])
    
    # print(f'ROTATION READING {R_marker_in_camera}\n')
    
    print(f'RAW READING {t_marker_in_camera}\n')
    


    R_marker_in_global = np.linalg.inv(R_marker_in_camera)  # This is a Rotation object

    # Build 4x4 transform for marker in camera frame
    T_marker_in_camera = np.eye(4)
    T_marker_in_camera[:3, :3] = R_marker_in_camera  # already a matrix
    T_marker_in_camera[:3, 3] = t_marker_in_camera.flatten()

    # Build 4x4 transform for marker in global frame
    T_marker_in_global = np.eye(4)

    # IMPORTANT FIX: convert Rotation object to matrix before assignment!
    T_marker_in_global[:3, :3] = R_marker_in_camera
    T_marker_in_global[:3, 3] = marker_pos.flatten()

    # Compute inverse of marker in camera frame transform
    inv = np.linalg.inv(T_marker_in_camera)

    # Compute camera pose in global frame
    T_camera_in_global = T_marker_in_global @ inv

    # Extract rotation and translation from camera pose in global frame
    R_camera_in_global = T_camera_in_global[:3, :3]
    t_camera_in_global = T_camera_in_global[:3, 3]

    displacement_vector = marker_pos - t_camera_in_global 

    # Yaw 
    yaw = math.atan2(R_camera_in_global[0, 1], R_camera_in_global[0, 0])
    # print(f'CALCULATED POSE {pos_world}\n')

    return t_camera_in_global, yaw



#CHATGPT
def angle_mean(angle1, angle2, alpha):
    # Convert angles to unit vectors
    x = alpha * np.cos(angle1) + (1 - alpha) * np.cos(angle2)
    y = alpha * np.sin(angle1) + (1 - alpha) * np.sin(angle2)
    return np.arctan2(y, x)
