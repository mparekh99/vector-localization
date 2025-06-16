import anki_vector
from anki_vector.util import degrees
import time
# from anki_vector.objects import CustomObjectTypes
from anki_vector.objects import LightCube
import numpy as np


# Taken from :https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/

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


def main():
    with anki_vector.Robot("00706c20", show_viewer=True) as robot:
        # Prepare Vector's posture
        robot.behavior.set_head_angle(degrees(5.0))
        robot.behavior.set_lift_height(0.0)

        # Enable custom object detection
        robot.vision.enable_custom_object_detection(True)
        print("🔍 Looking for custom objects...")

        try:
            while True:
                    for obj in robot.world.visible_objects:
                        if isinstance(obj, LightCube):
                            print("🎉 LightCube detected!")
                            pose = obj.pose
                            if pose:
                                print(f"📍 Pose relative to Vector:")
                                print(f"    x: {pose.position.x:.2f} mm")
                                print(f"    y: {pose.position.y:.2f} mm")
                                print(f"    z: {pose.position.z:.2f} mm")
                                print(f"    rotation (angle_z): {pose.rotation.angle_z.degrees:.2f}°")


                                print(f" Pose Relative Rotation to vector:")
                                print(f" q0: {pose.rotation.q0}")
                                print(f" q1: {pose.rotation.q1}")
                                print(f" q2: {pose.rotation.q2}")
                                print(f' q3: {pose.rotation.q3}')



                                print("CONVERT")

                                R = quaternion_rotation_matrix(pose.rotation)
                                t = np.array([[pose.position.x],
                                    [pose.position.y],
                                    [pose.position.z]])
                                
                                # Assemble 4x4 matrix

                                T_cube_in_vector = np.eye(4)
                                T_cube_in_vector[0:3, 0:3] = R
                                T_cube_in_vector[0:3, 3:4] = t


                                # STEP 3: Invert to get Vector Global position with respect to light cube at (0,0,0) 
                                R_inv = R.T                   # Transpose of R
                                t_inv = -np.dot(R_inv, t)     # -R^T * t

                                # Update Vector
                                
                                T_vector_in_cube = np.eye(4)
                                T_vector_in_cube[0:3, 0:3] = R_inv
                                T_vector_in_cube[0:3, 3:4] = t_inv


                                
                                # STEP 4 (Optional): Extract Vector's position
                                vector_global_position = T_vector_in_cube[0:3, 3]
                                print("\n🌍 Vector Global Position (x, y, z):")
                                print(vector_global_position.flatten())

                                # print(rot_matrix)

                            # robot.behavior.say_text("I see my LightCube!")
                            time.sleep(1)
                    time.sleep(0.2)

        except KeyboardInterrupt:
            print("🛑 Stopped by user.")

if __name__ == "__main__":
    main()


 