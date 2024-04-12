import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

'''
In this updated code:

camera_matrix and dist_coeffs are placeholders for the intrinsic parameters of the camera, which you should replace with
your actual calibrated values.
camera_to_robot_transform is the transformation matrix from the camera frame to the robot base frame, obtained from 
the EH calibration process.
The ArUco marker’s pose is estimated from the camera’s perspective and then transformed into the robot’s coordinate 
system using the camera_to_robot_transform matrix.
Please ensure that you replace the placeholders with the actual calibration results from the EH calibration process 
you’ve performed earlier. The transformation matrix camera_to_robot_transform should be the result of that calibration
process. The marker_length should also be set to the actual size of the ArUco marker used during the calibration.
'''
# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Load the camera parameters and transformation matrix
# These should be obtained through calibration
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
dist_coeffs = np.array([k1, k2, p1, p2, k3])
# The transformation matrix from the camera frame to the robot base frame
# This is the result of the EH calibration process
camera_to_robot_transform = np.array([[...]])  # 4x4 transformation matrix

# Define the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

try:
    while True:
        # Capture frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())

        # Detect ArUco marker and estimate pose
        corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, aruco_dict, parameters=aruco_params)
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Assume we have one marker and its pose is correctly estimated
        if ids is not None and len(ids) > 0:
            # Convert the marker pose to a 4x4 transformation matrix
            object_pose = np.eye(4)
            object_pose[:3, :3], _ = cv2.Rodrigues(rvec[0])
            object_pose[:3, 3] = tvec[0].flatten()

            # Transform the object pose to the robot base coordinate system
            object_pose_robot_base = np.dot(camera_to_robot_transform, object_pose)

            # Extract translation and rotation in the robot base coordinate system
            object_position = object_pose_robot_base[:3, 3]
            object_orientation = cv2.Rodrigues(object_pose_robot_base[:3, :3])[0]

            print(f"Object position in robot base: {object_position}")
            print(f"Object orientation in robot base: {object_orientation}")

        # Display the image with the detected marker
        aruco.drawDetectedMarkers(color_image, corners, ids)
        cv2.imshow('Frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
