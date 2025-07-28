import numpy as np
import math
from utils import quaternion_rotation_matrix

# ODOMETRY MOTION MODEL


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
        # roll,pitch,yaw = self.roll_pitch_yaw(mat[0:3, 0:3])

        # print(f'ROLL {roll} PITCH {pitch} YAW {yaw} \n')

        mat[0:3, 3] = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        return mat
    
    def extract_yaw(self, matrix):
        # Extract 2D yaw from rotation matrix
        return math.atan2(matrix[1, 0], matrix[0, 0])
    
    def reset(self, correct_pos, robo_pose):
        print("OOOOHOHOHOOHO")
        self.dr_pos = correct_pos
        self.last_robot_pose = robo_pose

    def roll_pitch_yaw(self, R): # Give back in radians YAW is on Z axis
        beta = np.arcsin(-R[1, 2])
        alpha = np.arctan2(R[0, 2], R[2, 2])
        gamma = np.arctan2(R[1, 0], R[0, 0])

        return gamma
    
     # https://www.youtube.com/watch?v=LrsTBWf6Wsc
    
    def update(self, robot, dt):

        if self.last_robot_pose is None:
            self.last_robot_pose = robot.pose
            return self.dr_pos, self.dr_yaw

        curr_matrix = self.pose_to_matrix(robot.pose)

        # curr_yaw = self.roll_pitch_yaw(curr_matrix[0:3, 0:3])

        # print(f"[Robot Pose] Yaw from transform = {curr_yaw:.4f} rad ({np.degrees(curr_yaw):.2f}°)\n")
        

        curr_global = self.transform @ curr_matrix
        last_global = self.transform @ self.pose_to_matrix(self.last_robot_pose)
        curr_yaw = self.roll_pitch_yaw(curr_global[0:3, 0:3])

        # CONSTANTS 
        r = 12.7  # mm -- wheel radius 
        R_w = 60.375  # mm distance between wheel and center refrence point
        # x, y, theta

        v_l = robot.left_wheel_speed_mmps
        v_r = robot.right_wheel_speed_mmps

        v = (v_l + v_r) / 2  # Give me velocity

        # ONLY USING YAW becuase i'm rotating only in z axis 

        x_hat = v * np.cos(curr_yaw) * dt
        y_hat = v * np.sin(curr_yaw) * dt
        curr_yaw = self.extract_yaw(curr_global)
        last_yaw = self.extract_yaw(last_global)
        dtheta = curr_yaw - last_yaw

        # curr_pos = curr_global[0:3, 3]
        # last_pos = last_global[0:3, 3]
        # delta_pos = curr_pos - last_pos
        # theta_hat = ((v_r - v_l)) / (2 * R_w) * dt

        # UPDATE
        self.dr_pos[0] += x_hat
        self.dr_pos[1] += y_hat

        self.dr_yaw += dtheta

        # self.dr_pos += delta_pos.reshape(3, 1)
        self.last_robot_pose = robot.pose

        # print(f'POSITION -> {self.dr_pos}, YAW: {self.dr_yaw}')

        return self.dr_pos, self.dr_yaw
    

# https://ursinusgraphics.github.io/RollPitchYaw/

