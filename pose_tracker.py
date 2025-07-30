import numpy as np
from dead_reckoning import DeadReckoning
from marker_processor import MarkerProcessor
import time


class PoseTracker:
    def __init__(self, start_pose, start_yaw, init_robot_pose, world):
        self.kalman = None ## WILL CHANGE WHEN I GET TO KALMAN 
        self.dead_reckoning = DeadReckoning(start_pose, start_yaw)
        self.marker_processor = MarkerProcessor(world)
        self.position = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.yaw = start_yaw
        self.last_observed_marker = None
        self.last_update_time = time.time()


    def new_marker_observed(self, marker_name):
        if self.last_observed_marker != marker_name:
            self.last_observed_marker = marker_name
            return True
        
        return False

    def update_from_moving(self, robot):
        current_time = time.time()
        dt = current_time - self.last_update_time  # in seconds
        self.last_update_time = current_time
        pos, yaw = self.dead_reckoning.update(robot, dt)
        self.position = pos
        self.yaw = yaw
    
    def update_from_marker(self, event, robo_pose):
        global_pos, global_yaw = self.marker_processor.process_marker(event, robo_pose)
        # UNCOMMET TO WORK
        if global_pos is not None:
            self.position = global_pos
            self.yaw = global_yaw
        #     self.yaw = global_yaw
            self.dead_reckoning.reset(global_pos, global_yaw)
        #     MAYBE UPDATE YAW -- TEST AND SEE
        return global_pos
        
    def get_x(self):
        return self.position[0].item()
    
    def get_y(self):
        return self.position[1].item()
    
    def get_yaw(self):
        return self.yaw
    
