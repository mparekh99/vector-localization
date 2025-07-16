import math
import numpy as np
from utils import scale_factor, frame_transformation


class MarkerProcessor:
    def __init__(self, world):
        self.world = world

    def process_marker(self, event):
        
        marker_info = self.world.marker_world.get(event.object_type)
        marker_name = marker_info["label"]
        axis = marker_info["axis"]

        marker_pos = marker_info["pos"]  # Grabs MARKER  Global POSE
        marker_rot = marker_info["rot"]  # Grabs MARKER GLOBAL Rotation I set

        # SCALE RAW DATA --- based on marker I set scale either x or y 
        # print(f'BEFORE SCALE -- {getattr(event.pose, axis)}')
        setattr(event.pose, axis, scale_factor(getattr(event.pose, axis), marker_name))

        print(f'MARKER SCALED POSE -> {event.pose.x, event.pose.y, event.pose.z}\n')


        # Homogenous Transformation + Inverse --> Frame Transformations to get vector pose from marker 
        pos_world = frame_transformation(event, marker_pos, marker_rot)

        #LAST SCALE
        if marker_name == "Circle":  # Because this one is flipped handling negatives would be same if I had a marker in the -y direction
            pos_world[1] = pos_world[1] - 200
        elif marker_name == "Diamond":
            pos_world[0] = pos_world[0] + 200
        elif marker_name == "Hexagon":
            pos_world[0] = pos_world[0] - 200
        
        position = np.array([pos_world[0], pos_world[1], pos_world[2]]).reshape(3, 1)
        # self.last_observed_marker = marker_name

        print(f'POSITION - {position[0], position[1]}\n')

        # Geometry calculating theta
        marker_yaw = np.arctan2(marker_pos[1] - position[1], marker_pos[0] - position[0])

        return position, marker_yaw
    