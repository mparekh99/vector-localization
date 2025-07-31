from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from anki_vector.util import degrees
import numpy as np


class World:
    
    def __init__(self, robot):

        self.robot = robot
        self.define_markers()
        self.set_head_and_lift()
        self.marker_world = self.define_marker_world()


    
    def define_markers(self):
        self.robot.vision.enable_custom_object_detection(True)
        
        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                            marker=CustomObjectMarkers.Diamonds4,
                                            size_mm=100,
                                            marker_width_mm=100,
                                            marker_height_mm=100)
        
        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                            marker=CustomObjectMarkers.Triangles5,
                                            size_mm=100,
                                            marker_width_mm=100,
                                            marker_height_mm=100)

        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType03,
                                            marker=CustomObjectMarkers.Triangles3,
                                            size_mm=100,
                                            marker_width_mm=100,
                                            marker_height_mm=100)

        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
        #                                     marker=CustomObjectMarkers.Diamonds4,
        #                                     size_mm=24,
        #                                     marker_width_mm=24,
        #                                     marker_height_mm=24)

        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
        #                                     marker=CustomObjectMarkers.Traingles5,
        #                                     size_mm=24,
        #                                     marker_width_mm=24,
        #                                     marker_height_mm=24)

        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType02,
        #                                     marker=CustomObjectMarkers.Traingles3,
        #                                     size_mm=24,
        #                                     marker_width_mm=24,
        #                                     marker_height_mm=24)


    def set_head_and_lift(self):
        self.robot.behavior.set_head_angle(degrees(7.0))
        self.robot.behavior.set_lift_height(0.0)
    

    def define_marker_world(self): 
        
        return {
            # Diamonds4 --> FRONT
            15 : {
                "pos": np.array([[0.0], [1400.0], [100.0]]),
                "label": "Front",
                "axis": 1,
                "translation": -120
            },
            # Triangles5 --> LEFT
            16: {
                "pos": np.array([[-1420.0], [0.0], [100.0]]),
                "label": "Left",
                "axis": 0,
                "translation": 120
            },
            # Triangles3 --> RIGHT
            18: {
                "pos": np.array([[1420.0], [0.0], [100.0]]),
                "label": "Right",
                "axis": 0,
                "translation": -120
            }
        }