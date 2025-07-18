import anki_vector
import threading
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
import numpy as np
import keyboard
import math
from anki_vector.events import Events
import csv
import os


import sys
import os

# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from world_setup import World
from pose_tracker import PoseTracker
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
    ax.set_xlim(-300, 300)
    ax.set_ylim(-100, 300)
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
    with anki_vector.Robot("006068a2", show_viewer=True) as robot:
        # plt.ion()
        # fig, ax = plt.subplots()

        world = World(robot)
        start_pose = [0, 0, 0]
        start_yaw = math.pi / 2
        pose_tracker = PoseTracker(start_pose, start_yaw, robot.pose, world)

        def on_robot_observed(robot, event_type, event):

            # marker_info = world.marker_world.get(event.object_type)
            # marker_name = marker_info["label"]
            # axis = marker_info["axis"]

            # marker_pos = marker_info["pos"]  # Grabs MARKER  Global POSE
            # marker_rot = marker_info["rot"]  # Grabs MARKER GLOBAL Rotation I set

            # pos_world = frame_transformation(event, marker_pos, marker_rot)

            raw_value = np.array([event.pose.x, event.pose.y]).flatten()

            csv_dir = r"C:\Users\mihpa\swarm\flocking\companion_cube\marker_detection_accuracy\data"
            os.makedirs(csv_dir, exist_ok=True)

            csv_filename = os.path.join(csv_dir, f"Triangle2_RAW.csv")

            file_exists = os.path.isfile(csv_filename)

            # Open in append mode and write header if creating
            with open(csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                if not file_exists:
                    writer.writerow(['RAW_X', 'RAW_Y', 'TRUE_X', 'TRUE_Y'])
                writer.writerow([
                    raw_value[0],
                    raw_value[1],
                    pose_tracker.get_x(),
                    pose_tracker.get_y()
                ])



            # Circle: shifted average RAW_X = 0.609, RAW_Y = 457.704
            # Diamond: shifted average RAW_X = -145.234, RAW_Y = -282.104
            # Hexagon: shifted average RAW_X = 189.885, RAW_Y = -284.601

            # if marker_name == "Circle":  # Because this one is flipped handling negatives would be same if I had a marker in the -y direction
            #     pos_world[0] = pos_world[0] - 0.609
            #     pos_world[1] = pos_world[1] - 457.704
            # elif marker_name == "Diamond":
            #     pos_world[0] = pos_world[0] + 145.234
            #     pos_world[1] = pos_world[1] + 282.104
            # elif marker_name == "Hexagon":
            #     pos_world[0] = pos_world[0] - 189.885
            #     pos_world[1] = pos_world[1] + 284.601


            # print(f'{pos_world[0], pos_world[1], pos_world[2]}')

                

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
