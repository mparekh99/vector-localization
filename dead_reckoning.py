
import numpy as np
import time 
import math
from pose import quaternion_rotation_matrix

 # Distance between wheels in mm
L = (5.87375 - 1.27) * 10 

class DeadReckoning:

    def __init__(self, init_pose, init_yaw, robo_pose):
        # 3D Position
        self.position = np.array(init_pose, dtype=np.float64).reshape(3, 1)
        self.yaw = init_yaw
        self.last_robot_pose = None
        self.transform = self.frame_transformation(robo_pose, init_pose, init_yaw)


    def frame_transformation(self, robo_pose, init_pose, init_yaw):
        # Robotframe matrix
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
    


    def update(self, curr_pose):  # CHATPGT

        if self.last_robot_pose is None:
            self.last_robot_pose = curr_pose
            return
        
        print(curr_pose.position.x, curr_pose.position.y, curr_pose.position.z)
        print(f'CURRENT YAW --> {self.yaw}')


        curr_global = self.transform @ self.pose_to_matrix(curr_pose)
        last_global = self.transform @ self.pose_to_matrix(self.last_robot_pose)

        curr_pos = curr_global[0:3, 3]
        last_pos = last_global[0:3, 3]
        delta_pos = curr_pos - last_pos

        curr_yaw = self.extract_yaw(curr_global)
        last_yaw = self.extract_yaw(last_global)
        dtheta = curr_yaw - last_yaw


        # Apply update
        self.position += delta_pos.reshape(3, 1)
        self.yaw += dtheta
        self.last_robot_pose = curr_pose

        print(f"Updated DR: pos={self.position}, yaw={self.yaw:.2f} rad")
        
        return 0

    def get_estimated_pose(self):
        pos = self.position[0:2, 0]
        return pos, self.yaw