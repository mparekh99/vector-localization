import anki_vector
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

        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
        #                                     marker=CustomObjectMarkers.Circles2,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)

        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
        #                                     marker=CustomObjectMarkers.Diamonds2,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)

        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType02,
        #                                     marker=CustomObjectMarkers.Hexagons2,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)
        
        # ### FOR TESTING 
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType03,
        #                                     marker=CustomObjectMarkers.Hexagons5,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)
            
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType04,
        #                                     marker=CustomObjectMarkers.Hexagons3,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)
        
        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType05,
                                            marker=CustomObjectMarkers.Triangles3,
                                            size_mm=25.4,
                                            marker_width_mm=25.4,
                                            marker_height_mm=25.4)
        
        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType06,
                                            marker=CustomObjectMarkers.Triangles5,
                                            size_mm=25.4,
                                            marker_width_mm=25.4,
                                            marker_height_mm=25.4)
        
        self.robot.world.define_custom_cube(CustomObjectTypes.CustomType07,
                                            marker=CustomObjectMarkers.Diamonds4,
                                            size_mm=25.4,
                                            marker_width_mm=25.4,
                                            marker_height_mm=25.4)
    
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType07,
        #                                     marker=CustomObjectMarkers.Diamonds5,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)
        
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType08,
        #                                     marker=CustomObjectMarkers.Circles5,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)
        
        # self.robot.world.define_custom_cube(CustomObjectTypes.CustomType09,
        #                                     marker=CustomObjectMarkers.Triangles2,
        #                                     size_mm=25.4,
        #                                     marker_width_mm=25.4,
        #                                     marker_height_mm=25.4)
            
    def set_head_and_lift(self):
        self.robot.behavior.set_head_angle(degrees(7.0))
        self.robot.behavior.set_lift_height(0.0)
    

    def define_marker_world(self): 
        
        return {
            # Diamonds4
            22: {
                "pos": np.array([[-200.0], [0.0], [0.0]]),
                "rot": self.rotation_z(90), # was -90
                "label": "Front",
                "axis": "x",
            },
            # Triangles5
            21: {
                "pos": np.array([[0.0], [200.0], [0.0]]),
                "rot": self.rotation_z(-90), # was be 90
                "label": "Left",
                "axis": "y",
            },
            # Triangles3
            20: {
                "pos": np.array([[200.0], [0.0], [0.0]]),
                "rot": self.rotation_z(90), # was 90
                "label": "Right",
                "axis": "y",
            }
        }

    # https://www.geeksforgeeks.org/computer-graphics/computer-graphics-3d-rotation-transformations/ 
    @staticmethod
    def rotation_z(degrees_angle):
        theta = np.radians(degrees_angle)
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])


