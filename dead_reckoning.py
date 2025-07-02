
import numpy as np

class DeadReckoning:

    def __init__(self):
        
        self.global_pose = np.array([0.0, 0.0])  # x, y in mm
        self.yaw = 0.0  # in radians
        self.last_robot_pose = None  # anki_vector.robot.pose

    
    def reset_with_marker(self, global_pose, yaw):
    
        self.global_pose = np.array(global_pose[:2])  # Just x and y
        self.yaw = yaw
        self.last_robot_pose = None  # discard old odometry info


    def update_with_odometry(self, current_robot_pose):

        # 2D handling 

        if self.last_robot_pose is None:
            self.last_robot_pose = current_robot_pose
            return  

        # Delta movement in robot frame
        dx = current_robot_pose.position.x - self.last_robot_pose.position.x
        dy = current_robot_pose.position.y - self.last_robot_pose.position.y
        dtheta = (current_robot_pose.rotation.angle_z - self.last_robot_pose.rotation.angle_z).radians

        # Converting to global coordinate system by using previous YAW I calulated before and is stored in this class

        delta_local = np.array([dx, dy])
        cos_theta = np.cos(self.yaw)
        sin_theta = np.sin(self.yaw)

        # PURE ROTATION 

        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])
        delta_world = rotation_matrix @ delta_local

        # Update pose estimate
        self.global_pose += delta_world
        self.yaw += dtheta
        self.last_robot_pose = current_robot_pose

    def get_estimated_pose(self):
        return self.global_pose, self.yaw

    






