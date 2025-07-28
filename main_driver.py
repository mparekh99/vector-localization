import anki_vector
import threading
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from world_setup import World
import numpy as np
import keyboard
import math
from anki_vector.events import Events
import csv
from pose_tracker import PoseTracker
import os
from utils import frame_transformation


#SCALE FACTOR Computed in Transform Accuracy
scale_factors = {"Front": 0.003010,
                 "Left": 0.002019, 
                 "Right": 0.002789}


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
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500, 500)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Vector and Marker Positions")
    ax.grid(True)

    # Plot markers
    for marker_type, marker_info in world.marker_world.items():
        x, y = marker_info["pos"][0], marker_info["pos"][1]
        ax.plot(x, y, 'ro')
        ax.text(x + 5, y + 5, f'{marker_info["label"]}', color='red', fontsize=8)

    # Plot Vector's position and heading
    # x = pose.get_dr_x()
    # y = pose.get_dr_y()

    x = pose.get_x()
    y = pose.get_y()
    ax.plot(x, y, 'bo')
    ax.text(x + 5, y + 5, "Vector", color='blue')

    length = 30

    dx = length * math.cos(pose.get_yaw())
    dy = length * math.sin(pose.get_yaw())

    # print(f'{x} -- {y} -- {dx} -- {dy}')

    ax.arrow(x, y, dx, dy, head_width=10, head_length=10, fc='blue', ec='blue')

def rotation_z(degrees_angle):
    theta = np.radians(degrees_angle)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])

def main():
    with anki_vector.Robot("00706c20") as robot:
        # plt.ion()
        # fig, ax = plt.subplots()

        world = World(robot)
        start_pose = [0, 0, 0]
        start_yaw = math.pi / 2
        pose_tracker = PoseTracker(start_pose, start_yaw, robot.pose, world)

        def on_robot_observed(robot, event_type, event):
            #SAME FRAME!!!!
            marker_info = world.marker_world.get(event.object_type)
            marker_name = marker_info["label"]
            marker_pos = marker_info["pos"].flatten()

            index = marker_info["axis"]
            translation = marker_info["translation"]


        #####FIXEDIIII!!!!!!!

            dx = event.pose.x - robot.pose.position.x
            dy = event.pose.y - robot.pose.position.y
            dz = event.pose.z - robot.pose.position.z

            d_xyz = np.array([dx, dy, dz])
            rot = rotation_z(90)

            update = rot @ d_xyz

            vector_in_global = marker_pos - update

            # Translation 

            vector_in_global[index] = vector_in_global[index] + translation
            print(vector_in_global)



            # Rotate vector world frame 90


            # print("ROBOT POSE: ", robot.pose)
            # print("EVENT POSE", event.pose.x, event.pose.y, event.pose.z)
            # print('\n')
            # print("ROBOT POSE", robot.pose)
            
            # print("ROBOT FRAME POSE: ", robot.pose)
            # marker_info = world.marker_world.get(event.object_type)
            # marker_name = marker_info["label"]

            # pose_tracker.update_from_marker(event, robot.pose)

            # if pose_tracker.new_marker_observed(marker_name):
            #     pose_tracker.update_from_marker(event, robot.pose)
            # else:
            #     pose_tracker.update_from_moving(robot.pose)


        robot.events.subscribe(on_robot_observed, Events.robot_observed_object)

        # Start teleop thread
        listener_thread = threading.Thread(target=teleop_listener, daemon=True)
        listener_thread.start()

        while True:
            # # Update dead reckoning
            # print(robot.pose.quaternion)
            # pose_tracker.update_from_moving(robot)
            # print("ROBOT POSE", robot.pose)

            # #Plot
            # plot_scene(ax, pose_tracker, world)
            # plt.draw()
            # plt.pause(0.1)

            # Teleop
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


                

if __name__ == "__main__":
    main()
