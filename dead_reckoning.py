import numpy as np

# ODOMETRY MOTION MODEL with wheel speeds and gyroscope


class DeadReckoning:
    def __init__(self, start_pose, start_yaw):
        self.dr_pos = np.array(start_pose, dtype=np.float64).reshape(3, 1)
        self.dr_yaw = start_yaw
    
    def reset(self, correct_pos, correct_yaw):
        self.dr_pos = correct_pos
        self.dr_yaw = correct_yaw
     # https://www.youtube.com/watch?v=LrsTBWf6Wsc
    
    def update(self, robot, dt):
        dtheta = robot.gyro.z * dt
        self.dr_yaw += dtheta

        v_l = robot.left_wheel_speed_mmps
        v_r = robot.right_wheel_speed_mmps

        v = (v_l + v_r) / 2 

        x_hat = v * np.cos(self.dr_yaw) * dt
        y_hat = v * np.sin(self.dr_yaw) * dt


        # UPDATE
        self.dr_pos[0] += x_hat
        self.dr_pos[1] += y_hat

        return self.dr_pos, self.dr_yaw
    

# https://ursinusgraphics.github.io/RollPitchYaw/

