import anki_vector
from anki_vector.util import degrees
from anki_vector.objects import CustomObjectMarkers, CustomObjectTypes
import time
import numpy as np

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

        robot.vision.enable_custom_object_detection(True)
        robot.world.define_custom_cube(custom_object_type=CustomObjectTypes.CustomType00,
                                    marker=CustomObjectMarkers.Circles2,
                                    size_mm=20.0,
                                    marker_width_mm=23.0, marker_height_mm=23.0)
        
        print("Looking for custom cube...")

        # Set head and lift to ideal scanning pose
        robot.behavior.set_head_angle(degrees(7.0))
        robot.behavior.set_lift_height(0.0)

        try:
            while True:
                for obj in robot.world.visible_custom_objects:

                    # print(f"Custom object seen with archetype: {obj.archetype}")
                    # print(f"Custom object seen with name: {obj.descriptive_name}")
                    # print(f"Custom object seen with id: {obj.object_id}")

                    pose = obj.pose
                    position = pose.position
                    rotation = pose.rotation

                    # print(f"Pose (x, y, z): {position.x:.2f}, {position.y:.2f}, {position.z:.2f}")
                    # print(f"Rotation (quaternion): {rotation.q0:.3f}, {rotation.q1:.3f}, {rotation.q2:.3f}, {rotation.q3:.3f}")
                    # print("-" * 40)

                    # print("CONVERT!!!")

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
                    print("\n")



                time.sleep(1)

        except KeyboardInterrupt:
            print("Stopped.")


if __name__ == "__main__":
    main()
