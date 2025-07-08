
import numpy as np
import time 
import math

 # Distance between wheels in mm
L = (5.87375 - 1.27) * 10 


class DeadReckoning:

    def __init__(self, init_pose, init_yaw):
        # 2D Position
        self.position = init_pose
        self.yaw = init_yaw
        self.last_robot_pose = None 

    def update(self, curr_pose):  # CHATPGT
        
        if self.last_robot_pose is None:
            self.last_robot_pose = curr_pose
            return 
        
        # Caluclate change in robot frame
        dx = curr_pose.position.x - self.last_robot_pose.position.x
        dy = curr_pose.position.y - self.last_robot_pose.position.y
        dtheta = (curr_pose.rotation.angle_z.radians - self.last_robot_pose.rotation.angle_z.radians)

        # Rotate delta to global frame using previous yaw
        # https://en.wikipedia.org/wiki/Rotation_matrix - 2D Pure ROtation
        cos_theta = math.cos(self.yaw)
        sin_theta = math.sin(self.yaw)
        
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])

        delta_local = np.array([dx, dy])
        delta_world = rotation_matrix @ delta_local

        # Update global pose and yaw
        self.position += delta_world
        self.yaw += dtheta
        self.last_robot_pose = curr_pose

    def get_estimated_pose(self):
        return self.position, self.yaw

    






