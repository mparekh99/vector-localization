import anki_vector
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from world_setup import World
from vector_localizer import VectorLocalizer
from pose_estimator import PoseEstimator
import numpy as np
import keyboard

def main():

    pos_world = None
    yaw = None

    

    with anki_vector.Robot("006068a2") as robot:

        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-300, 300)
        ax.set_ylim(-100, 300)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("Vector and Marker Positions")
        ax.grid(True)
        
        world = World(robot)

        localizer = VectorLocalizer(world.marker_world_poses)
        
        while True:

            found_marker = False

            for obj in robot.world.visible_custom_objects:
                pos_world, yaw = localizer.get_pose(obj)

                # print(f"POSE -> {pos_world}\n\n")
                
                if pos_world is not None and yaw is not None:
                    # Set pose_estimator privates here 
                    # estimator.reset_with_marker(pos_world, heading_vector)
                    found_marker = True
                    break   # Saves me from looping more than I need to
            
            if found_marker is False:  # No custom objects were found
                
                print("IDK")

            ax.clear()
            ax.set_xlim(-300, 300)
            ax.set_ylim(-100, 300)
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_title("Vector and Marker Positions")
            ax.grid(True)

            # Draw markers
            for marker_type, marker_info in localizer.marker_world_poses.items():
                x, y = marker_info["pos"][0][0], marker_info["pos"][1][0]
                ax.plot(x, y, 'ro')
                
                if marker_type == CustomObjectMarkers.Circles2:
                    label = "Circles2"
                elif marker_type == CustomObjectMarkers.Diamonds2:
                    label = "Diamonds2"
                elif marker_type == CustomObjectMarkers.Hexagons2:
                    label = "Hexagons2"
                

            if pos_world is not None and yaw is not None:
                x, y = pos_world[0], pos_world[1]
                ax.plot(x, y, 'bo')
                ax.text(x + 5, y + 5, "Vector", color='blue')
                
                # Arrow length (small, just for orientation)
                length = 30

                # Direction components from yaw
                # HAD TO MAKE YAW NEGATIVE B/C of Matplot
                # Matplot does 0 radians rightward and increases counter clockwise


                dx = length * np.cos(-yaw)
                dy = length * np.sin(-yaw)
                # print(dx, dy)

                # Draw arrow indicating heading
                ax.arrow(x, y, dx, dy, head_width=10, head_length=10, fc='blue', ec='blue')


            plt.pause(0.1)
            time.sleep(1)


if __name__ == "__main__":
    main()
