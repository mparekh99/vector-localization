import anki_vector
import threading
import time
import matplotlib.pyplot as plt
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
from world_setup import World
from vector_localizer import VectorLocalizer, angle_mean
from dead_reckoning import DeadReckoning
import numpy as np
import keyboard
import math
from pose_fix import Pose
from anki_vector.events import Events
import csv
from utils import scale_factor


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

    dx = length * math.cos(pose.dr_yaw)
    dy = length * math.sin(pose.dr_yaw)

    # print(f'{x} -- {y} -- {dx} -- {dy}')

    ax.arrow(x, y, dx, dy, head_width=10, head_length=10, fc='blue', ec='blue')


def main():
    with anki_vector.Robot("00706c20") as robot:
        plt.ion()
        fig, ax = plt.subplots()

        world = World(robot)
        start_pose = [0, 0, 0]
        start_yaw = math.pi / 2
        pose = Pose(start_pose, start_yaw, robot.pose)

        def on_robot_observed(robot, event_type, event):
            
            print(event.pose.x, event.pose.y)
            
            marker_info = world.marker_world.get(event.object_type)
            marker_name = marker_info["label"]
            axis = marker_info["axis"]

            marker_pos = pose.update_pos(event, world, robot)
            yaw_curr = robot.pose.rotation.angle_z.radians
            pose.update_yaw(marker_pos, yaw_curr)

        robot.events.subscribe(on_robot_observed, Events.robot_observed_object)

        # Start teleop thread
        listener_thread = threading.Thread(target=teleop_listener, daemon=True)
        listener_thread.start()

        while True:
            # # Update dead reckoning
            # pose.dead_reckoning(robot.pose)

            # print(robot.pose)

            # # #Plot
            plot_scene(ax, pose, world)
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
