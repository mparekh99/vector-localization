import math
import numpy as np
from utils import frame_transformation, scale_transform_point
import csv
import os


class MarkerProcessor:
    def __init__(self, world):
        self.world = world

    def process_marker(self, event):
        
        marker_info = self.world.marker_world.get(event.object_type)
        marker_name = marker_info["label"]
        axis = marker_info["axis"]

        marker_pos = marker_info["pos"]  # Grabs MARKER  Global POSE
        marker_rot = marker_info["rot"]  # Grabs MARKER GLOBAL Rotation I set


        # Homogenous Transformation + Inverse --> Frame Transformations to get vector pose from marker 
        pos_world, yaw = frame_transformation(event, marker_pos, marker_rot)

        pos_world[0], pos_world[1] = scale_transform_point(pos_world[0], pos_world[1], marker_name)

        print(f'POSITION --> {pos_world.flatten()}')
        
        #UNCOMMET FOR RUNNING
        # setattr(event.pose, axis, scale_factor(getattr(event.pose, axis), marker_name))


        # Geometry calculating theta
        

        return pos_world, yaw
    