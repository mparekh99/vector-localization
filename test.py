import anki_vector
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from world_setup import World
from vector_localizer import VectorLocalizer
from dead_reckoning import DeadReckoning
import numpy as np
import keyboard


def live_plotter(ax, global_pose, yaw, localizer):

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
        

    if global_pose is not None and yaw is not None:
        x, y = global_pose[0], global_pose[1]
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



def main():

    global_pose = None
    yaw = None

    with anki_vector.Robot("00706c20") as robot:

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

        dead_reckoning = DeadReckoning()

        while True:



            if keyboard.is_pressed("up"):
                robot.motors.set_wheel_motors(100, 100)  # move forward
            elif keyboard.is_pressed("down"):
                robot.motors.set_wheel_motors(-100, -100)  # move backward
            elif keyboard.is_pressed("left"):
                robot.motors.set_wheel_motors(-50, 50)  # turn left
            elif keyboard.is_pressed("right"):
                robot.motors.set_wheel_motors(50, -50)  # turn right
            else:
                robot.motors.set_wheel_motors(0, 0) 





            found_marker = False

            for obj in robot.world.visible_custom_objects:
                global_pose, yaw = localizer.get_pose(obj)
                
                if global_pose is not None and yaw is not None:
                    # Set pose_estimator privates here 
                    # dead_reckoning.reset_with_marker(global_pose, yaw)
                    found_marker = True
                    break   # Saves me from looping more than I need to
            
            if found_marker is False:  # No custom objects were found
                
                print("IDK")
                # dead_reckoning.update_with_odometry(robot.pose)
                # global_pose, yaw = dead_reckoning.get_estimated_pose()

            print(f"GLOBAL POSE --> {global_pose}")

            # dead_reckoning.update_with_odometry(robot.pose)


            # print(global_pose)
            # print(yaw)

            live_plotter(ax, global_pose, yaw, localizer)


            plt.pause(0.1)
            time.sleep(0.1)


if __name__ == "__main__":
    main()
