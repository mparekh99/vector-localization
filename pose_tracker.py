import numpy as np
from dead_reckoning import DeadReckoning
from marker_processor import MarkerProcessor

class PoseTracker:
    def __init__(self, start_pose, start_yaw, init_robot_pose, world):
        self.kalman = None ## WILL CHANGE WHEN I GET TO KALMAN 
        self.dead_reckoning = DeadReckoning(start_pose, start_yaw, init_robot_pose)
        self.marker_processor = MarkerProcessor(world)
        self.position = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.yaw = start_yaw
        self.last_observed_marker = None

    def new_marker_observed(self, marker_name):
        if self.last_observed_marker != marker_name:
            self.last_observed_marker = marker_name
            return True
        
        return False

    def update_from_moving(self, curr_pose):
        pos, yaw = self.dead_reckoning.update(curr_pose)
        self.position = pos
        self.yaw = yaw
    
    def update_from_marker(self, event, robo_pose):
        global_pos, global_yaw = self.marker_processor.process_marker(event)
        if global_pos is not None:
            self.position = global_pos
            self.yaw = global_yaw
            self.dead_reckoning.reset(global_pos, global_yaw, robo_pose)
            # MAYBE UPDATE YAW -- TEST AND SEE
        
    def get_x(self):
        return self.position[0, 0].item()
    
    def get_y(self):
        return self.position[1, 0].item()
    
    def get_yaw(self):
        return self.yaw
    
