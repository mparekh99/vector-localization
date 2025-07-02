import numpy as np
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
import joblib
import math


## SENSOR STUFF
scaler = joblib.load('dist_calibrated.pkl')


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

## Got from CHATGPT 

def scaling(y_reported):
    return scaler.predict([[y_reported]])[0]



# https://lavalle.pl/planning/node103.html
class VectorLocalizer:
    def __init__(self, marker_world_poses):
        self.marker_world_poses = marker_world_poses

    def get_pose(self, obj):
        # Match marker type
 
        if obj.archetype.custom_type == CustomObjectTypes.CustomType00:
            marker_type = CustomObjectMarkers.Circles2
            index = 1  # Change scale y coordinate
        elif obj.archetype.custom_type == CustomObjectTypes.CustomType01:
            marker_type = CustomObjectMarkers.Diamonds2
            index = 0  # Scale x coordinate
        elif obj.archetype.custom_type == CustomObjectTypes.CustomType02:
            marker_type = CustomObjectMarkers.Hexagons2
            index = 0  # scale x coordinate
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


        if marker_type is not CustomObjectMarkers.Circles2:


            marker_pos = self.marker_world_poses[marker_type]["pos"][0:2].flatten()
            circle_pos = self.marker_world_poses[CustomObjectMarkers.Circles2]["pos"][0:2].flatten()
            robot_pos = pos_world[0:2].flatten()


            # Robot -> Non-Circle-Marker
            u = marker_pos - robot_pos
            # Robot -> Circle Marker
            v = circle_pos - robot_pos

            proj = (np.dot(u, v) / (np.sqrt(v[0] ** 2 + v[1] ** 2) ** 2)) * v

            # print(u,v)

            # print("proj:", proj, "proj[1]:", proj[1], "type:", type(proj[1]))


            temp = np.array([proj[0], 200 - scaling(proj[1])])

            
            # Project u_new on v_new --> projecting update pos to Current

            u_new = circle_pos - temp

            v_new = marker_pos - temp

            # unit_vector = v / (math.sqrt(v[0] ** 2 + v[1]** 2))

            # # r = starting point + scaled_dist * direction to circle
            # r = self.marker_world_poses[marker_type]["pos"][0:2] + scaled_dist * unit_vector



            # Project Back to desired after scaling (FLIP)

            update_proj = (np.dot(v_new, u_new) / (np.sqrt(u_new[0] ** 2 + u_new[1] ** 2) ** 2)) * u_new


            pos_world[0] = update_proj[0]
            pos_world[1] = update_proj[1]





            # Project to Circles Marker Y 
            # u = pos_world[0:2] - self.marker_world_poses[marker_type]["pos"][0:2]

            # # https://www.varsitytutors.com/precalculus-help/find-a-direction-vector-when-given-two-points --- finding directional vector
            # # NEed to start from terminal point in this case circle
            # v = self.marker_world_poses[CustomObjectMarkers.Circles2]["pos"][0:2] - self.marker_world_poses[marker_type]["pos"][0:2]

            # # Projection of vector onto another - https://www.youtube.com/watch?v=5AhWoO4IPGM 
            # proj = (np.dot(u, v) / (math.sqrt(v[0] ** 2 + v[1] ** 2) ** 2)) * v

            # # print(proj)

            # scaled_dist = 200 - scaling(float(proj[1]))  # Because I kept getting errors 


            # # Parametric Line Equation - https://www.google.com/search?q=parametric+equations+of+a+line+using+direction+vector+formula&rlz=1C1CHBF_enUS1122US1122&oq=parametric+equations+of+a+line+using+direction+vector+formula&gs_lcrp=EgZjaHJvbWUyCQgAEEUYORigATIHCAEQIRigATIHCAIQIRigAdIBCDI2NzdqMGo3qAIAsAIA&sourceid=chrome&ie=UTF-8#fpstate=ive&vld=cid:5e3daf9e,vid:PyPp4QvQY3Q,st:0 
            # # In projection frame
        
            # unit_vector = v / (math.sqrt(v[0] ** 2 + v[1]** 2))

            # # r = starting point + scaled_dist * direction to circle
            # r = self.marker_world_poses[marker_type]["pos"][0:2] + scaled_dist * unit_vector

            # # Project found projection onto robot -> marker vector 

            # robot_marker = self.marker_world_poses[marker_type]["pos"][0:2] - pos_world[0:2]

            # update_proj = (np.dot(r, robot_marker) / (math.sqrt(robot_marker[0] ** 2 + robot_marker[1] ** 2) ** 2)) * robot_marker

            # pos_world[0] = update_proj[0]
            # pos_world[1] = update_proj[1]

            # print(scaled_pose)
        
        else:

            pos_world[1] = 200 - scaling(pos_world[1])


        # FIND YAW 
        yaw = np.arctan2(global_pose[0][1], global_pose[0][0])

        return pos_world, yaw