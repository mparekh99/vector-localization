import anki_vector
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from world_setup import World
from vector_localizer import VectorLocalizer
from pose_estimator import PoseEstimator
import numpy as np

def main():

    pos_world = None
    heading_vector = None
    angle = None

    with anki_vector.Robot("006068a2") as robot:
        world = World(robot)
        # print(world.marker_world_poses)
        localizer = VectorLocalizer(world.marker_world_poses)

        estimator = PoseEstimator()
    

        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlim(-300, 300)
        ax.set_ylim(-100, 300)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title("Vector and Marker Positions")
        ax.grid(True)
        
        while True:


            found_marker = False

            for obj in robot.world.visible_custom_objects:
                pos_world, heading_vector, angle = localizer.get_pose(obj)
                
                if pos_world is not None and heading_vector is not None and angle is not None:
                    # Set pose_estimator privates here 
                    estimator.reset_with_marker(pos_world, heading_vector)
                    found_marker = True
                    break   # Saves me from looping more than I need to
            
            if found_marker is False:  # No custom objects were found
                
                estimator.update_with_odometry(robot.pose)

                pos_world, heading_vector = estimator.get_estimated_pose()
                
            #PLOTTTING

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

            if pos_world is not None:
                vx, vy = pos_world[0], pos_world[1]
                ax.plot(vx, vy, 'bo')
                ax.text(vx + 5, vy + 5, "Vector", color='blue')
                # Heading vector in world (already computed earlier)

                # Plot arrow    
                hx, hy = heading_vector[0, 0], heading_vector[1, 0]

                # Normalize and scale for visualization
                heading_length = 30  # length of arrow in mm
                norm = np.linalg.norm([hx, hy])
                if norm != 0:
                    hx = (hx / norm) * heading_length
                    hy = (hy / norm) * heading_length

                    # Draw arrow indicating heading
                    ax.arrow(vx, vy, hx, hy, head_width=10, head_length=10, fc='blue', ec='blue')

            plt.pause(0.1)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
