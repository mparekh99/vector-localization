import numpy as np
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes



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




# https://lavalle.pl/planning/node103.html
class VectorLocalizer:
    def __init__(self, marker_world_poses):
        self.marker_world_poses = marker_world_poses

    def get_pose(self, obj):
        # Match marker type

        # TODO Prioritize Distance 
        # TODO Revise IF ELSE if 2 are in view 
        if obj.archetype.custom_type == CustomObjectTypes.CustomType00:
            marker_type = CustomObjectMarkers.Circles2
        elif obj.archetype.custom_type == CustomObjectTypes.CustomType01:
            marker_type = CustomObjectMarkers.Diamonds2
        elif obj.archetype.custom_type == CustomObjectTypes.CustomType02:
            marker_type = CustomObjectMarkers.Hexagons2
        else:
            return None, None

        # Known marker world pose
        marker_pos = self.marker_world_poses[marker_type]["pos"]  # Grabs MARKER  Global POSE
        marker_rot = self.marker_world_poses[marker_type]["rot"]  # Grabs MARKER GLOBAL Rotation I set

        ## Build Homogenous Transofrmation Matrix 
        marker_matrix = np.eye(4)
        marker_matrix[0:3, 0:3] = marker_rot
        marker_matrix[0:3, 3:4] = marker_pos

        # Detected Marker
        R = quaternion_rotation_matrix(obj.pose.rotation)
        t = np.array([[obj.pose.position.x],
                      [obj.pose.position.y],
                      [obj.pose.position.z]])
        
        # SCALE
        scale_factor = 200.0 / 254.5  
        t = scale_factor * t

        # Build Homogenous Transformation Matrix of Detection
        detected_matrix = np.eye(4)
        detected_matrix[0:3, 0:3] = R  
        detected_matrix[0:3, 3:4] = t

        # FInd pose in global coordinate system
        global_pose = marker_matrix @ np.linalg.inv(detected_matrix)

        pos_world = global_pose[0:3, 3]

        # FIND YAW 
        yaw = np.arctan2(global_pose[0][1], global_pose[0][0])

        return pos_world, yaw