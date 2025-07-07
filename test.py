import anki_vector
import threading
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from world_setup import World
from vector_localizer import VectorLocalizer
from dead_reckoning import DeadReckoning
import numpy as np
import keyboard
import math
from pose import Pose



#TODO HANDLE POSE 
#  To make it simpler I'll say I know the vector's hard coded start pose.

# Breaking down the problem, Knowing Global Pose on Start : Loop this 
#1) I CAN NOT see the marker, DEAD RECKOINING - Until below 
# 2) I CAN see the marker -- relocalize/ update 



#TODO FIX TELEOP
#TODO FIX ORIENTATION

#CHATGPT
control_state = {
    "left": False,
    "right": False,
    "forward": False,
    "backward": False
}

#CHATGPT
def teleop_listener():
    while True:
        control_state["forward"] = keyboard.is_pressed("up")
        control_state["backward"] = keyboard.is_pressed("down")
        control_state["left"] = keyboard.is_pressed("left")
        control_state["right"] = keyboard.is_pressed("right")
        time.sleep(0.01)  # Small delay to reduce CPU load



def plot_scene(ax, pose, world):
    ax.clear()
    ax.set_xlim(-300, 300)
    ax.set_ylim(-100, 300)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Vector and Marker Positions")
    ax.grid(True)

    # Plot markers
    for marker_type, marker_info in world.marker_world_poses.items():
        x, y = marker_info["pos"][0][0], marker_info["pos"][1][0]
        ax.plot(x, y, 'ro')
        ax.text(x + 5, y + 5, f'{marker_info["label"]}', color='red', fontsize=8)

    # Plot Vector's position and heading
    x = pose.position[0]
    y = pose.position[1]
    ax.plot(x, y, 'bo')
    ax.text(x + 5, y + 5, "Vector", color='blue')

    length = 30

    dx = length * math.cos(pose.curr_yaw)
    dy = length * math.sin(pose.curr_yaw)

    print(f'{x} -- {y} -- {dx} -- {dy}')

    ax.arrow(x, y, dx, dy, head_width=10, head_length=10, fc='blue', ec='blue')




def main():

    with anki_vector.Robot("00706c20") as robot:

        plt.ion()
        fig, ax = plt.subplots()
        
        world = World(robot)

        # STORE STARTING YAW
        ## FOR NOW HARD CODE IT IN RADIANS == Makes it simpler to start 

        start_pose = [0, 0]
        # Pointing at Circles marker (0,200)
        start_yaw = math.pi / 2

        pose = Pose(start_pose, start_yaw)

        listener_thread = threading.Thread(target=teleop_listener, daemon=True)
        listener_thread.start()


        while True:
            #PLOT MATPLOT 
            plot_scene(ax, pose, world)
            plt.draw()
            plt.pause(0.1)


            #CHATGPT -- Lets me not deal with blocking for realtime manuevering
            if control_state["forward"]:
                robot.motors.set_wheel_motors(100, 100)
            elif control_state["backward"]:
                robot.motors.set_wheel_motors(-100, -100)
            elif control_state["left"]:
                robot.motors.set_wheel_motors(-50, 50)
            elif control_state["right"]:
                robot.motors.set_wheel_motors(50, -50)
            else:
                robot.motors.set_wheel_motors(0, 0)



            marker_found = False

            for obj in robot.world.visible_custom_objects: # #TODO PROF asked for 8 markers using 3 at the moment 
                # If found :
                # Update GLOBAL POSE!!!!
                marker_pos = pose.update_pos(obj, world)
                yaw_curr = robot.pose.rotation.angle_z.radians
                pose.update_yaw(obj, marker_pos, yaw_curr)
                marker_found = True
                break

            # if marker_found == False:
            #     # OBJECT was NOT FOUND!!

            
            

            




        # localizer = VectorLocalizer(world.marker_world_poses, world.marker_map, robot.pose.rotation.angle_z)

        # # dead_reckoning = DeadReckoning()

        # while True:


            




        #     if keyboard.is_pressed("up"):
        #         robot.motors.set_wheel_motors(100, 100)  # move forward
        #     elif keyboard.is_pressed("down"):
        #         robot.motors.set_wheel_motors(-100, -100)  # move backward
        #     elif keyboard.is_pressed("left"):
        #         robot.motors.set_wheel_motors(-50, 50)  # turn left
        #     elif keyboard.is_pressed("right"):
        #         robot.motors.set_wheel_motors(50, -50)  # turn right
        #     else:
        #         robot.motors.set_wheel_motors(0, 0) 


        #     found_marker = False

        #     for obj in robot.world.visible_custom_objects:
        #         global_pose, yaw = localizer.get_pose(obj, robot.pose.rotation.angle_z)

        #         if global_pose is not None and yaw is not None:
        #             # Set pose_estimator privates here 
        #             # dead_reckoning.reset_with_marker(global_pose, yaw) # when visible
        #             found_marker = True
        #             break   # Saves me from looping more than I need to
            
        #     if found_marker is False:  # No custom objects were found
                
        #         print("IDK")
        #         # dead_reckoning.update_with_odometry(robot.pose)
        #         # global_pose, yaw = dead_reckoning.get_estimated_pose()

        #     print(f"GLOBAL POSE --> {global_pose}")

        #     # dead_reckoning.update_with_odometry(robot.pose)


        #     # print(global_pose)
        #     # print(yaw)

        #     print(f'FINAL YAW --> {yaw}')

        #     live_plotter(ax, global_pose, yaw, localizer)


        #     plt.pause(0.1)
        #     time.sleep(0.1)


if __name__ == "__main__":
    main()
