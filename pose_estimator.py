
import numpy as np

class PoseEstimator:

    def __init__(self):
        
        self.position = np.array([0.0, 0.0])  # x, y in mm
        self.heading = 0.0  # in radians
        self.last_robot_pose = None  # anki_vector.robot.pose

    
    def reset_with_marker(self, position_world, heading_vector):
    
        # self.position = position_world
        self.position = np.array(position_world[:2])  # Just x and y
        self.heading = np.arctan2(heading_vector[1, 0], heading_vector[0, 0]) # Gets me theta 
        self.last_robot_pose = None  # discard old odometry info


    def update_with_odometry(self, current_robot_pose):

        if self.last_robot_pose is None:
            self.last_robot_pose = current_robot_pose
            return  

        # Delta movement in robot frame
        dx = current_robot_pose.position.x - self.last_robot_pose.position.x
        dy = current_robot_pose.position.y - self.last_robot_pose.position.y
        dtheta = (current_robot_pose.rotation.angle_z - self.last_robot_pose.rotation.angle_z).radians

        # Convert to world frame using current heading
        delta_local = np.array([dx, dy])
        cos_theta = np.cos(self.heading)
        sin_theta = np.sin(self.heading)

        # PURE ROTATION 

        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])
        delta_world = rotation_matrix @ delta_local

        # Update pose estimate
        self.position += delta_world
        self.heading += dtheta
        self.last_robot_pose = current_robot_pose

    def get_estimated_pose(self):
        # Return the current estimate and heading as a unit vector
        heading_vector = np.array([
            [np.cos(self.heading)],
            [np.sin(self.heading)]
        ])
        return self.position, heading_vector

    






