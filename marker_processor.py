import math
import numpy as np
from utils import quaternion_rotation_matrix
import csv
import os
from quanterion import Quaternion


def rotation_z(degrees_angle):
    theta = np.radians(degrees_angle)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])

class MarkerProcessor:
    def __init__(self, world):
        self.world = world

    def process_marker(self, event, robot_pose):
        
        marker_info = self.world.marker_world.get(event.object_type)
        marker_name = marker_info["label"]
        index = marker_info["axis"]

        marker_pos = marker_info["pos"].flatten()  # Grabs MARKER  Global POSE
        translation = marker_info["translation"]


        dx = event.pose.x - robot_pose.position.x
        dy = event.pose.y - robot_pose.position.y
        dz = event.pose.z - robot_pose.position.z

        d_xyz = np.array([dx, dy, dz])

        
        rot = rotation_z(90)

        update = rot @ d_xyz

        vector_in_global = marker_pos - update
        
        # Translation 

        # vector_in_global[index] = vector_in_global[index] + translation
        print(vector_in_global)
    

        quat = Quaternion(robot_pose.rotation.q0, robot_pose.rotation.q1, robot_pose.rotation.q2, robot_pose.rotation.q3)
        q = quaternion_rotation_matrix(quat)

        matrix = rot @ q

        print(matrix)
        global_yaw = math.atan2(matrix[1, 0], matrix[0, 0])

        return vector_in_global, global_yaw
    