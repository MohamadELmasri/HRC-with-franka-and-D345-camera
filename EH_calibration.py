import rospy
import pandas as pd
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
import cv2
import numpy as np
import pyrealsense2 as rs

''' In this updated code, the perform_calibration function now includes the steps to convert 
the collected gripper poses and camera poses into the format required by OpenCV’s calibrateHandEye function,
which performs the actual calibration. The results are printed out at the end of the calibration process.

Please ensure that you have the correct values for marker_size, camera_matrix, and dist_coeffs,
which are placeholders for the actual size of the ArUco marker and the camera calibration parameters 
specific to your D345 camera. These values are crucial for the accuracy of the calibration.
'''
# Function to convert pose to a dictionary
def pose_to_dict(pose):
    return {
        'px': pose.position.x,
        'py': pose.position.y,
        'pz': pose.position.z,
        'ox': pose.orientation.x,
        'oy': pose.orientation.y,
        'oz': pose.orientation.z,
        'ow': pose.orientation.w
    }

# Callback function for the robot state
def state_callback(data):
    # Extract the pose of the gripper from the FrankaState message
    pose = data.O_T_EE  # This is a 4x4 transformation matrix in row-major order

    # Convert the 4x4 matrix to a Pose message
    pose_msg = PoseStamped()
    pose_msg.pose.position.x = pose[12]
    pose_msg.pose.position.y = pose[13]
    pose_msg.pose.position.z = pose[14]

    # Convert the rotation matrix to a quaternion
    quaternion = tf_trans.quaternion_from_matrix(pose)
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]

    # Convert pose to dictionary and append to the list
    pose_dict = pose_to_dict(pose_msg.pose)
    gripper_poses.append(pose_dict)

    # Print the current gripper pose
    print(pose_dict)

# Function to save the recorded poses to a CSV file
def save_poses_to_csv(file_path):
    df = pd.DataFrame(gripper_poses)
    df.to_csv(file_path, index=False)
    print(f"Saved gripper poses to {file_path}")

# List to store the gripper poses
gripper_poses = []

# Initialize the camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Function to perform eye-on-hand calibration
def perform_calibration():
    # Collect a series of gripper poses and corresponding images
    collected_data = []
    try:
        while len(collected_data) < 20:  # Collect 20 samples
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Detect ArUco markers in the image
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, aruco_dict, parameters=aruco_params)

            # If markers are detected, proceed to calculate the pose
            if len(corners) > 0:
                # Assume we know the size of the marker and the camera parameters
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

                # Store the gripper pose and the camera pose
                if len(gripper_poses) > 0:
                    collected_data.append((gripper_poses[-1], {'rvec': rvec[0], 'tvec': tvec[0]}))

            # Display the image
            cv2.imshow('Frame', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    # Perform the actual calibration using the collected data
    robot_poses = []
    camera_rvecs = []
    camera_tvecs = []

    for data in collected_data:
        gripper_pose, camera_pose = data

        # Convert gripper pose to transformation matrix
        gripper_matrix = tf_trans.quaternion_matrix([
            gripper_pose['ox'],
            gripper_pose['oy'],
            gripper_pose['oz'],
            gripper_pose['ow']
        ])
        gripper_matrix[:3, 3] = [
            gripper_pose['px'],
            gripper_pose['py'],
            gripper_pose['pz']
        ]
        robot_poses.append(gripper_matrix)

        # Camera pose (from ArUco marker detection)
        camera_rvecs.append(camera_pose['rvec'])
        camera_tvecs.append(camera_pose['tvec'])

    # Convert rotation vectors to rotation matrices
    camera_rotations = [cv2.Rodrigues(rvec)[0] for rvec in camera_rvecs]

    # Perform the hand-eye calibration
    retval, camera_to_robot, robot_to_camera = cv2.calibrateHandEye(
        robot_poses, camera_rotations, camera_tvecs,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    # camera_to_robot is the transformation matrix from the camera to the robot's end-effector
    # robot_to_camera is the transformation matrix from the robot's end-effector to the camera

    # Print out the results
    print("Camera to Robot Transformation Matrix:")
    print(camera_to_robot)

    print("Robot to Camera Transformation Matrix:")
    print(robot_to_camera)

# Main function
def main():
    rospy.init_node('gripper_pose_recorder')

    # Subscribe to the FrankaState topic to get the robot state
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, state_callback)

    # Start the calibration process
    perform_calibration()

    # Save the recorded poses to a CSV file
    save_poses_to_csv('gripper_poses.csv')

if __name__ == '__main__':
    main()
