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
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-500, 1500)
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


def main():
    # output_file_path = "marker10_log.csv"
    with anki_vector.Robot("006068a2") as robot:

        plt.ion()
        fig, ax = plt.subplots()

        world = World(robot)
        start_pose = [0, 0, 0]
        start_yaw = math.pi / 2
        pose_tracker = PoseTracker(start_pose, start_yaw, robot.pose, world)

        def on_robot_observed(robot, event_type, event):
            # print("HELLo")
            # print(f"Event received: {event.object_type}")

            pose_tracker.update_from_marker(event, robot.pose)
            # x = pose_tracker.get_x()
            # y = pose_tracker.get_y()
            # writer.writerow([x, y])
            # else:
            #     pose_tracker.update_from_moving(robot.pose)


        robot.events.subscribe(on_robot_observed, Events.robot_observed_object)

        # Start teleop thread
        listener_thread = threading.Thread(target=teleop_listener, daemon=True)
        listener_thread.start()

        while True:
            # # Update dead reckoning
            # print(robot.pose.quaternion)
            pose_tracker.update_from_moving(robot)
            # print("ROBOT POSE", robot.pose)

            # #Plot
            plot_scene(ax, pose_tracker, world)
            plt.draw()
            plt.pause(0.1)

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
