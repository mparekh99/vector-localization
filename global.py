import anki_vector
from anki_vector.util import degrees
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
import time
import numpy as np
import matplotlib.pyplot as plt


def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q.q0
    q1 = Q.q1
    q2 = Q.q2
    q3 = Q.q3
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def rotation_z(degrees_angle):
    theta = np.radians(degrees_angle)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])




def main():

    # Known marker poses (position + rotation)
    marker_world_poses = {
        CustomObjectMarkers.Circles2: {
            "pos": np.array([[0.0], [200.0], [0.0]]),    # (x, y, z) mm
            "rot": rotation_z(90)                       # Facing -Y
        },
        CustomObjectMarkers.Diamonds2: {
            "pos": np.array([[-200.0], [0.0], [0.0]]),
            "rot": rotation_z(180)                        # Facing -X
        }, 
        CustomObjectMarkers.Hexagons2: {
            "pos": np.array([[200.0], [0.0], [0.0]]),
            "rot": rotation_z(0)
        }
    }

        # Setup real-time plot
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-300, 300)
    ax.set_ylim(-100, 300)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Vector and Marker Positions")
    ax.grid(True)

    with anki_vector.Robot("006068a2") as robot:

        robot.vision.enable_custom_object_detection(True)
        
        # Marker 1
        robot.world.define_custom_cube(custom_object_type=CustomObjectTypes.CustomType00,
                                    marker=CustomObjectMarkers.Circles2,
                                    size_mm=23.0,
                                    marker_width_mm=23.0, marker_height_mm=23.0)
        # Marker 2

        robot.world.define_custom_cube(custom_object_type=CustomObjectTypes.CustomType01,
                                    marker=CustomObjectMarkers.Diamonds2,
                                    size_mm=23.0,
                                    marker_width_mm=23.0, marker_height_mm=23.0)
        
        # Marker 3 

        robot.world.define_custom_cube(custom_object_type=CustomObjectTypes.CustomType02,
                                    marker=CustomObjectMarkers.Hexagons2,
                                    size_mm=23.0,
                                    marker_width_mm=23.0, marker_height_mm=23.0)
        
        
        print("Looking for custom cube...")

        # Set head and lift to ideal scanning pose
        robot.behavior.set_head_angle(degrees(7.0))
        robot.behavior.set_lift_height(0.0)

        try:
            while True:
                vector_position_world = None
        
                for obj in robot.world.visible_custom_objects:

                    print(obj.archetype.custom_type)

                    if obj.archetype.custom_type == CustomObjectTypes.CustomType00:
                        marker_type = CustomObjectMarkers.Circles2
                    elif obj.archetype.custom_type == CustomObjectTypes.CustomType01:
                        marker_type = CustomObjectMarkers.Diamonds2
                    elif obj.archetype.custom_type == CustomObjectTypes.CustomType02:
                        marker_type = CustomObjectMarkers.Hexagons2
                    else:
                        continue


            
                    # Get marker pose in world (position and rotation)
                    marker_pos_world = marker_world_poses[marker_type]["pos"]
                    marker_rot_world = marker_world_poses[marker_type]["rot"]
      
                    T_cube_in_world = np.eye(4)
                    T_cube_in_world[0:3, 0:3] = marker_rot_world
                    T_cube_in_world[0:3, 3:4] = marker_pos_world
                  
                    # Get observed marker pose in Vector frame
                    pose = obj.pose
                    R = quaternion_rotation_matrix(pose.rotation)
                    t = np.array([[pose.position.x],
                                  [pose.position.y],
                                  [pose.position.z]])
                
                    T_cube_in_vector = np.eye(4)
                    T_cube_in_vector[0:3, 0:3] = R
                    T_cube_in_vector[0:3, 3:4] = t

                    # Invert to get Vector pose in cube frame
                    T_vector_in_cube = np.linalg.inv(T_cube_in_vector)

                    # Final transform: Vector in world   matrix multiplication
                    T_vector_in_world = T_cube_in_world @ T_vector_in_cube
                    vector_position_world = T_vector_in_world[0:3, 3]

                    # Heading 
                    robot_forward_vector = np.array([[1], [0], [0]])
                    R_world = T_vector_in_world[0:3, 0:3]
                    heading_vector_world = R_world @ robot_forward_vector

                    # Extract X and Y components of heading
                    hx, hy = heading_vector_world[0, 0], heading_vector_world[1, 0]

                    # Calculate angle in degrees (0° = +X, counter-clockwise)
                    heading_angle_rad = np.arctan2(hy, hx)
                    heading_angle_deg = np.degrees(heading_angle_rad)

                    # Normalize to [0, 360)
                    if heading_angle_deg < 0:
                        heading_angle_deg += 360

                    print(f"🧭 Vector heading: {heading_angle_deg:.1f}° relative to +X (global frame)")


                    print(f"\n🌍 Vector Global Position (x, y, z):")
                    print(vector_position_world.flatten())

                # Plot
                ax.clear()
                ax.set_xlim(-300, 300)
                ax.set_ylim(-100, 300)
                ax.set_xlabel("X (mm)")
                ax.set_ylabel("Y (mm)")
                ax.set_title("Vector and Marker Positions")
                ax.grid(True)

                # Plot markers
                for marker_type, marker_info in marker_world_poses.items():
                    x, y = marker_info["pos"][0][0], marker_info["pos"][1][0]
                    ax.plot(x, y, 'ro')
                    
                    if marker_type == CustomObjectMarkers.Circles2:
                        label = "Circles2"
                    elif marker_type == CustomObjectMarkers.Diamonds2:
                        label = "Diamonds2"
                    elif marker_type == CustomObjectMarkers.Hexagons2:
                        label = "Hexagons2"

                
                    ax.text(x + 5, y + 5, label, color='red')

                # Plot Vector
                if vector_position_world is not None:
                    vx, vy = vector_position_world[0], vector_position_world[1]
                    ax.plot(vx, vy, 'bo')
                    ax.text(vx + 5, vy + 5, "Vector", color='blue')
                    # Heading vector in world (already computed earlier)

                    # Plot arrow    
                    hx, hy = heading_vector_world[0, 0], heading_vector_world[1, 0]

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

        except KeyboardInterrupt:
            print("Stopped.")
            plt.ioff()
            plt.show()


if __name__ == "__main__":
    main()
