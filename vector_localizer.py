import numpy as np
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes

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

class VectorLocalizer:
    def __init__(self, marker_world_poses):
        self.marker_world_poses = marker_world_poses

    def get_pose(self, obj):
        # Match marker type
        if obj.archetype.custom_type == CustomObjectTypes.CustomType00:
            marker_type = CustomObjectMarkers.Circles2
        elif obj.archetype.custom_type == CustomObjectTypes.CustomType01:
            marker_type = CustomObjectMarkers.Diamonds2
        elif obj.archetype.custom_type == CustomObjectTypes.CustomType02:
            marker_type = CustomObjectMarkers.Hexagons2
        else:
            return None, None, None

        # Known marker world pose
        marker_pos = self.marker_world_poses[marker_type]["pos"]
        marker_rot = self.marker_world_poses[marker_type]["rot"]
        T_cube_world = np.eye(4)
        T_cube_world[0:3, 0:3] = marker_rot
        T_cube_world[0:3, 3:4] = marker_pos

        # Observed cube pose in Vector's frame
        R = quaternion_rotation_matrix(obj.pose.rotation)
        t = np.array([[obj.pose.position.x],
                      [obj.pose.position.y],
                      [obj.pose.position.z]])

        T_cube_vector = np.eye(4)
        T_cube_vector[0:3, 0:3] = R  
        T_cube_vector[0:3, 3:4] = t

        # Compute Vector pose in world
        T_vector_cube = np.linalg.inv(T_cube_vector)
        T_vector_world = T_cube_world @ T_vector_cube

        pos_world = T_vector_world[0:3, 3]

        # Heading vector
        forward = np.array([[1], [0], [0]])
        heading_vector = T_vector_world[0:3, 0:3] @ forward
        hx, hy = heading_vector[0, 0], heading_vector[1, 0]
        angle = np.degrees(np.arctan2(hy, hx))
        if angle < 0:
            angle += 360

        return pos_world, heading_vector, angle
