import anki_vector
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from anki_vector.util import degrees
import numpy as np
from scipy.spatial.transform import Rotation as R


class World:
    
    def __init__(self, robot):

        self.robot = robot
        self.define_markers()
        self.set_head_and_lift()
        self.marker_world = self.define_marker_world()


    
    def define_markers(self):
        self.robot.vision.enable_custom_object_detection(True)
        
        result = self.robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                            marker=CustomObjectMarkers.Diamonds4,
                                            size_mm=100,
                                            marker_width_mm=100,
                                            marker_height_mm=100) 
        print(result)
        

        # print("HELLO")
        
        result = self.robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                            marker=CustomObjectMarkers.Triangles5,
                                            size_mm=100,
                                            marker_width_mm=100,
                                            marker_height_mm=100) 
        
        print(result)
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType13,
        #                                     marker=CustomObjectMarkers.Triangles5,
        #                                     size_mm=24,
        #                                     marker_width_mm=24,
        #                                     marker_height_mm=24) 
        

        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType03,
                                            marker=CustomObjectMarkers.Triangles3,
                                            size_mm=100,
                                            marker_width_mm=100,
                                            marker_height_mm=100) 
        
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
        #                                     marker=CustomObjectMarkers.Diamonds4,
        #                                     size_mm=24,
        #                                     marker_width_mm=24,
        #                                     marker_height_mm=24) 
    

            
    def set_head_and_lift(self):
        self.robot.behavior.set_head_angle(degrees(7.0))
        self.robot.behavior.set_lift_height(0.0)
    

    def define_marker_world(self): 
        
        return {
            # Diamonds4
            15 : {
                "pos": np.array([[0.0], [1420.0], [100.0]]),
                "label": "Front",
                "axis": 1,
                "translation": 106
            },
            # Triangles5
            16: {
                "pos": np.array([[-1420.0], [0.0], [100.0]]),
                "label": "Left",
                "axis": 0,
                "translation": -77
            },
            # Triangles3
            18: {
                "pos": np.array([[1420.0], [0.0], [100.0]]),
                "label": "Right",
                "axis": 0,
                "translation": 77
            }
        }

    # https://www.geeksforgeeks.org/computer-graphics/computer-graphics-3d-rotation-transformations/ 
    # @staticmethod
    # def rotation_z(degrees_angle):
    #     theta = np.radians(degrees_angle)
    #     return np.array([
    #         [np.cos(theta), -np.sin(theta), 0],
    #         [np.sin(theta),  np.cos(theta), 0],
    #         [0,              0,             1]
    #     ])
    # @staticmethod
    # def rotation_y(degrees_angle):
    #     theta = np.radians(degrees_angle)
    #     return np.array([
    #         [np.cos(theta), 0, np.sin(theta)],
    #         [0,             1,             0],
    #         [-np.sin(theta),0, np.cos(theta)]
    #     ])
    # @staticmethod
    # def rotation_x(degrees_angle):
    #     theta = np.radians(degrees_angle)
    #     return np.array([
    #         [1,              0,             0]
    #         [0, np.cos(theta), -np.sin(theta)],
    #         [0, np.sin(theta),  np.cos(theta)],
    #     ])


