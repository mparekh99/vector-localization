# 2D GLOBAL POSE OBJECT
import warnings
import numpy as np
import joblib
import math
from utils import quaternion_rotation_matrix, frame_transformation, scale_factor, angle_mean




# TODO ADD DEADRECKONING WITHIN CLASS
class Pose:

    def __init__(self, start_pose, start_yaw, robo_pose):
        self.position = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.dr_pos = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.dr_yaw = start_yaw
        self.last_robot_pose = None
        self.transform = self.dr_frame_transform(robo_pose, start_pose, start_yaw)
        self.start_yaw = start_yaw
        # self.position = start_pose
        # self.start_yaw = start_yaw
        # self.curr_yaw = 0

    def get_dr_x(self):
        return self.dr_pos[0, 0].item()
    
    def get_dr_y(self):
        return self.dr_pos[1, 0].item()


    def get_x(self):
        return self.position[0, 0].item()
    
    def get_y(self):
        return self.position[1, 0].item()


    def dr_frame_transform(self, robo_pose, init_pose, init_yaw):
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
    

    # HANDLES DEAD RECKONING FROM (0,0) looking at pi /2 reference 
    def dead_reckoning(self, curr_pose):  # CHATPGT

        if self.last_robot_pose is None:
            self.last_robot_pose = curr_pose
            return
        
        # print(curr_pose.position.x, curr_pose.position.y, curr_pose.position.z)
        # print(f'CURRENT YAW --> {self.yaw}')


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



        # print(f"Updated DR: pos={self.dr_pos}, yaw={self.dr_yaw:.2f} rad")
        
        return 0
    
    def extract_yaw(self, matrix):
        # Extract 2D yaw from rotation matrix
        return math.atan2(matrix[1, 0], matrix[0, 0])

    def update_pos(self, event, world, robot):

        marker_info = world.marker_world.get(event.object_type)
        
        marker_type = marker_info["marker_type"]
        model_label = marker_info["label"]
        axis = marker_info["axis"]
        constant = marker_info["constant"]


        marker_pos = marker_info["pos"]  # Grabs MARKER  Global POSE
        marker_rot = marker_info["rot"]  # Grabs MARKER GLOBAL Rotation I set

        # SCALE RAW DATA --- based on marker I set scale either x or y 
        print(f'BEFORE SCALE -- {getattr(event.pose, axis)}')
        setattr(event.pose, axis, scale_factor(getattr(event.pose, axis)))


        # Homogenous Transformation + Inverse --> Frame Transformations to get vector pose from marker 
        pos_world = frame_transformation(event, marker_pos, marker_rot)

        #LAST SCALE
        # if model_label == "Circle":  # Because this one is flipped handling negatives would be same if I had a marker in the -y direction
        #     pos_world[1] = pos_world[1]
        # elif model_label == "Diamond":
        #     pos_world[0] = pos_world[0] + 200
        # elif model_label == "Hexagon":
        #     pos_world[0] = pos_world[0] - 200

        
        self.position = np.array([pos_world[0], pos_world[1], pos_world[2]]).reshape(3, 1)

        print(f'POSITION - {self.position[0], self.position[1]}\n')

        return marker_pos




    def update_yaw(self, obj, marker_pos, curr_yaw):
        # Geometry calculating theta
        marker_yaw = np.arctan2(marker_pos[1] - self.position[1], marker_pos[0] - self.position[0])

        odometry_yaw = self.start_yaw - curr_yaw

        self.curr_yaw = angle_mean(marker_yaw, odometry_yaw, 0.7).item()  # Bc it returns 1d array 

        
        return 
