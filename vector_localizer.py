import numpy as np
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
import joblib
import math


## Loading my non linear regression models from the data I collected on each marker
scaling_models = joblib.load('pose_calibration_models.pkl')



# CHATGPT
def normalize_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


#CHATGPT
def angle_mean(angle1, angle2, alpha=0.7):
    # Convert angles to unit vectors
    x = alpha * np.cos(angle1) + (1 - alpha) * np.cos(angle2)
    y = alpha * np.sin(angle1) + (1 - alpha) * np.sin(angle2)
    return np.arctan2(y, x)




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
    def __init__(self, marker_world_poses, marker_map, initial_yaw):
        self.marker_world_poses = marker_world_poses
        self.marker_map = marker_map
        self.initial_yaw = initial_yaw
        self.yaw_offset = None 

    def get_pose(self, obj, yaw_curr):
        # Match marker type
        
        # Getter because its a dictionary in python
        marker_info = self.marker_map.get(obj.archetype.custom_type)  # Check for valid Marker 
        if marker_info is None:
            return None, None
        
        marker_type = marker_info["marker_type"]
        model_label = marker_info["model_label"]
        axis = marker_info["axis"]

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
        

        # print("Detected Rotation Matrix:")
        # print(R)
        
        # # Scale b/c I'm consistently getting underscaled values -- measured at 100mm and got 51.87 mm reading
        # scale_factor = 100.0 / 51.87 
        # t = scale_factor * t


        # Build Homogenous Transformation Matrix of Detection
        detected_matrix = np.eye(4)
        detected_matrix[0:3, 0:3] = R  
        detected_matrix[0:3, 3:4] = t

        np.set_printoptions(precision=2, suppress=True)

        inv_detected = np.linalg.inv(detected_matrix)

        # print(inv_detected)

        # FInd pose in global coordinate system
        global_pose = marker_matrix @ inv_detected

        pos_world = global_pose[0:3, 3]

        if model_label == "Diamonds2":  # Because this one is flipped handling negatives would be same if I had a marker in the -y direction
            pos_world[axis] = scaling_models[model_label].predict([[pos_world[axis]]])[0] - 200
        else:
            pos_world[axis] = 200 - scaling_models[model_label].predict([[pos_world[axis]]])[0]


        # FIND YAW 
        # yaw = np.arctan2(global_pose[0][1], global_pose[0][0])

        # yaw = obj.pose.rotation.angle_z

        # GEOMETRY 

        # # GLOBAL YAW
        # yaw = np.arctan((marker_pos[0] - pos_world[0]) / (marker_pos[1] - pos_world[1]))
        ## CHATGPT Suggestion

        marker_yaw = np.arctan2(marker_pos[1] - pos_world[1], marker_pos[0] - pos_world[0])

        marker_yaw = normalize_angle(marker_yaw)

        yaw_curr_rad = normalize_angle(yaw_curr.radians)

        ## Setting yaw offset so ODOMETRY CAN BE USEFUL for GLOBAL USE 
        if self.yaw_offset == None:
            self.yaw_offset = normalize_angle(marker_yaw - self.initial_yaw.radians)
        

        # print(f'LAST YAW --> {self.yaw_offset} -  YAW CURR --> {yaw_curr}')
        # REINFORCE 
        odometry_yaw = normalize_angle(yaw_curr_rad + self.yaw_offset)

        print(f'YAW OFFSET --> {self.yaw_offset} = MARKER_YAW {marker_yaw} -  INITIAL  --> {self.initial_yaw.radians}')

        print(f'ODOMETRY --> {odometry_yaw} -  YAW CURR --> {yaw_curr.radians}\n\n')


        # print(f'ODOMETRY YAW: {odometry_yaw}\n MARKER YAW: {marker_yaw}\n')

        alpha = .7

        fused_yaw = angle_mean(marker_yaw, odometry_yaw, alpha)

        return pos_world, float(fused_yaw)