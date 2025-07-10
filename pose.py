# 2D GLOBAL POSE OBJECT
import warnings
import numpy as np
import joblib


## Loading my non linear regression models from the data I collected on each marker
scaling_models = joblib.load('pose_calibration_models.pkl')


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

def frame_transformation(obj, marker_pos, marker_rot):
    ## Build Homogenous Transofrmation Matrix 
    marker_matrix = np.eye(4)
    marker_matrix[0:3, 0:3] = marker_rot
    marker_matrix[0:3, 3:4] = marker_pos

    # Detected Marker
    R = quaternion_rotation_matrix(obj.pose.rotation)
    t = np.array([[obj.pose.position.x],
                [obj.pose.position.y],
                [obj.pose.position.z]])
    
    # Build Homogenous Transformation Matrix of Detection
    detected_matrix = np.eye(4)
    detected_matrix[0:3, 0:3] = R  
    detected_matrix[0:3, 3:4] = t

    # Inverse it
    inv_detected = np.linalg.inv(detected_matrix)

    # Find pose in global coordinate system
    global_pose = marker_matrix @ inv_detected

    pos_world = global_pose[0:3, 3]

    return pos_world

#CHATGPT
def angle_mean(angle1, angle2, alpha):
    # Convert angles to unit vectors
    x = alpha * np.cos(angle1) + (1 - alpha) * np.cos(angle2)
    y = alpha * np.sin(angle1) + (1 - alpha) * np.sin(angle2)
    return np.arctan2(y, x)


class Pose:

    def __init__(self, start_pose, start_yaw):
        self.position = start_pose
        self.start_yaw = start_yaw
        self.curr_yaw = 0

    def update_pos(self, obj, world):
        # GET is built into python no need to make function
        marker_info = world.marker_map.get(obj.archetype.custom_type)  # Check for valid Marker 

        if marker_info is None:
            warnings.warn("Not able to retrieve marker info.", UserWarning)
            return
        
        marker_type = marker_info["marker_type"]
        model_label = marker_info["model_label"]
        axis = marker_info["axis"]


        marker_pos = world.marker_world_poses[marker_type]["pos"]  # Grabs MARKER  Global POSE
        marker_rot = world.marker_world_poses[marker_type]["rot"]  # Grabs MARKER GLOBAL Rotation I set

        pos_world = frame_transformation(obj, marker_pos, marker_rot) # Homogenous Transformation + Inverse --> Frame Transformations to get vector pose from marker 

        # SCALING 
        if model_label == "Diamonds2":  # Because this one is flipped handling negatives would be same if I had a marker in the -y direction
            pos_world[axis] = scaling_models[model_label].predict([[pos_world[axis]]])[0] - 200
        else:
            pos_world[axis] = 200 - scaling_models[model_label].predict([[pos_world[axis]]])[0]

        # Scaled more? 
        
        # Update-- Grab 2D pose b/c z remains constant 
        
        self.position = np.array([pos_world[0], pos_world[1]])

        ## REINFORCE

        


        print(f'Position --> {self.position}')


        return marker_pos


    def update_yaw(self, obj, marker_pos, curr_yaw):
        # Geometry calculating theta
        marker_yaw = np.arctan2(marker_pos[1] - self.position[1], marker_pos[0] - self.position[0])

        odometry_yaw = self.start_yaw - curr_yaw

        self.curr_yaw = angle_mean(marker_yaw, odometry_yaw, 0.7).item()  # Bc it returns 1d array 

        
        return 
