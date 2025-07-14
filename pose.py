import warnings
import numpy as np
import joblib
import math
import time
from utils import quaternion_rotation_matrix, frame_transformation, scale_factor, angle_mean

# CHATGPT FIx
class Pose:
    def __init__(self, start_pose, start_yaw, robo_pose):
        self.position = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.dr_pos = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.dr_yaw = start_yaw
        self.last_robot_pose = None
        self.last_time = None
        self.transform = self.dr_frame_transform(robo_pose, start_pose, start_yaw)
        self.start_yaw = start_yaw

    def get_dr_x(self):
        return self.dr_pos[0, 0].item()

    def get_dr_y(self):
        return self.dr_pos[1, 0].item()

    def get_x(self):
        return self.position[0, 0].item()

    def get_y(self):
        return self.position[1, 0].item()

    def dr_frame_transform(self, robo_pose, init_pose, init_yaw):
        robot_matrix = np.eye(4)
        robot_matrix[0:3, 0:3] = quaternion_rotation_matrix(robo_pose.rotation)
        robot_matrix[0:3, 3] = np.array([
            robo_pose.position.x,
            robo_pose.position.y,
            robo_pose.position.z
        ])

        global_matrix = np.eye(4)
        global_matrix[0:3, 0:3] = np.array([
            [math.cos(init_yaw), -math.sin(init_yaw), 0],
            [math.sin(init_yaw),  math.cos(init_yaw), 0],
            [0,                   0,                   1]
        ])
        global_matrix[0:3, 3] = init_pose

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

    def dead_reckoning_with_gyro(self, curr_pose, robot, dt):
        if self.last_robot_pose is None or self.last_time is None:
            self.last_robot_pose = curr_pose
            self.last_time = time.time()
            return

        # 1. Get position and yaw change from pose
        curr_global = self.transform @ self.pose_to_matrix(curr_pose)
        last_global = self.transform @ self.pose_to_matrix(self.last_robot_pose)

        curr_pos = curr_global[0:3, 3]
        last_pos = last_global[0:3, 3]
        delta_pos = curr_pos - last_pos
        distance = np.linalg.norm(delta_pos[0:2])  # only XY plane

        # 2. Integrate gyro yaw (Z-axis angular velocity)
        # gyro in rad/s, dt in seconds → Δθ = ω * dt
        gyro_z = robot.gyro.z  # rad/s
        dtheta = gyro_z * dt
        self.dr_yaw += dtheta

        # 3. If distance moved is significant, update position
        movement_threshold = 1e-3
        if distance > movement_threshold:
            theta = self.dr_yaw
            dx = delta_pos[0]
            dy = delta_pos[1]

            # Rotate motion vector into world frame
            world_dx = dx * np.cos(theta) - dy * np.sin(theta)
            world_dy = dx * np.sin(theta) + dy * np.cos(theta)

            self.dr_pos[0] += world_dx
            self.dr_pos[1] += world_dy

        self.last_robot_pose = curr_pose
        self.last_time = time.time()

    def extract_yaw(self, matrix):
        return math.atan2(matrix[1, 0], matrix[0, 0])

    def update_pos(self, event, world, robot):
        marker_info = world.marker_world.get(event.object_type)
        marker_name = marker_info["label"]
        axis = marker_info["axis"]

        marker_pos = marker_info["pos"]
        marker_rot = marker_info["rot"]

        print(f'BEFORE SCALE -- {getattr(event.pose, axis)}')
        setattr(event.pose, axis, scale_factor(getattr(event.pose, axis), marker_name))

        pos_world = frame_transformation(event, marker_pos, marker_rot)

        # Handle axis corrections
        if marker_name == "Circle":
            pos_world[1] -= 200
        elif marker_name == "Diamond":
            pos_world[0] += 200
        elif marker_name == "Hexagon":
            pos_world[0] -= 200

        self.position = np.array([pos_world[0], pos_world[1], pos_world[2]]).reshape(3, 1)

        print(f'POSITION - {self.position[0], self.position[1]}\n')
        return marker_pos

    def update_yaw(self, obj, marker_pos, curr_yaw):
        marker_yaw = np.arctan2(marker_pos[1] - self.position[1], marker_pos[0] - self.position[0])
        odometry_yaw = self.start_yaw - curr_yaw
        self.curr_yaw = angle_mean(marker_yaw, odometry_yaw, 0.7).item()
