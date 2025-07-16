import numpy as np
import math
from utils import quaternion_rotation_matrix


class DeadReckoning:
    def __init__(self, start_pose, start_yaw, robo_pose):
        self.dr_pos = np.array(start_pose, dtype=np.float64).reshape(3,1)
        self.dr_yaw = start_yaw
        self.last_robot_pose = None
        self.transform = self.dr_frame_transform(robo_pose, start_pose, start_yaw)

    def dr_frame_transform(self, robo_pose, init_pose, init_yaw):
        
        robot_matrix = np.eye(4)
        robot_matrix[0:3, 0:3] = quaternion_rotation_matrix(robo_pose.rotation)
        robot_matrix[0:3, 3] = np.array([
            robo_pose.position.x,
            robo_pose.position.y,
            robo_pose.position.z
        ])

        # Global frame of starting postiion (0,0,0)
        global_matrix = np.eye(4)
        global_matrix[0:3, 0:3] = np.array([
            [math.cos(init_yaw), -math.sin(init_yaw), 0],
            [math.sin(init_yaw),  math.cos(init_yaw), 0],
            [0,                   0,                   1]
        ])
        global_matrix[0:3, 3] = init_pose

        # Compute transform from robot frame to world frame
        transform = global_matrix @ np.linalg.inv(robot_matrix)
        return transform
    
    def pose_to_matrix(self, pose):

        # print(pose)

        mat = np.eye(4)
        mat[0:3, 0:3] = quaternion_rotation_matrix(pose.rotation)
        mat[0:3, 3] = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        return mat
    
    def extract_yaw(self, matrix):
        # Extract 2D yaw from rotation matrix
        return math.atan2(matrix[1, 0], matrix[0, 0])
    
    def reset(self, correct_pos, correct_yaw, robo_pose):
        self.dr_pos = correct_pos
        self.dr_yaw = correct_yaw
        self.last_robot_pose = robo_pose

    
    def update(self, curr_pose):
        if self.last_robot_pose is None:
            self.last_robot_pose = curr_pose
            return self.dr_pos, self.dr_yaw
        
        curr_global = self.transform @ self.pose_to_matrix(curr_pose)
        last_global = self.transform @ self.pose_to_matrix(self.last_robot_pose)

        curr_pos = curr_global[0:3, 3]
        last_pos = last_global[0:3, 3]
        delta_pos = curr_pos - last_pos

        curr_yaw = self.extract_yaw(curr_global)
        last_yaw = self.extract_yaw(last_global)
        dtheta = curr_yaw - last_yaw


        # Apply update
        self.dr_pos += delta_pos.reshape(3, 1)
        self.dr_yaw += dtheta
        self.last_robot_pose = curr_pose

        return self.dr_pos, self.dr_yaw