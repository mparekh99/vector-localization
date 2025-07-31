import math
import numpy as np
from utils import quaternion_rotation_matrix, rotation_z

from quanterion import Quaternion


class MarkerProcessor:
    def __init__(self, world):
        self.world = world

    def process_marker(self, event, robot_pose):
        marker_info = self.world.marker_world.get(event.object_type)
        index = marker_info["axis"]

        marker_pos = marker_info["pos"].flatten()  # Grabs MARKER  Global POSE
        translation = marker_info["translation"]

        # I CAN DO THIS B/C event pose and robot pose in same world frame

        dx = event.pose.x - robot_pose.position.x
        dy = event.pose.y - robot_pose.position.y
        dz = event.pose.z - robot_pose.position.z

        d_xyz = np.array([dx, dy, dz])
        rot = rotation_z(90)

        update = rot @ d_xyz

        vector_in_global = marker_pos - update
        # Translation
        vector_in_global[index] = vector_in_global[index] + translation
        print(vector_in_global)
    

        quat = Quaternion(robot_pose.rotation.q0, robot_pose.rotation.q1, robot_pose.rotation.q2, robot_pose.rotation.q3)
        q = quaternion_rotation_matrix(quat)

        matrix = rot @ q

        # GET HEADING
        global_yaw = math.atan2(matrix[1, 0], matrix[0, 0])

        return vector_in_global, global_yaw
    