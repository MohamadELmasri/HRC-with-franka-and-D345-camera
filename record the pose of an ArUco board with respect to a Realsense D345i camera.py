
'''
In this code:

Replace fx, fy, cx, cy, k1, k2, p1, p2, and k3 with the actual camera matrix and distortion coefficients obtained from
your camera calibration.
Adjust markersX, markersY, markerLength, and markerSeparation to match the actual ArUco board you are using.
This script will display the video feed from the camera and draw the detected markers and axes on the board. 
It will also print out the rotation and translation vectors representing the pose of the board with respect to the 
camera.

Make sure you have the pyrealsense2 and opencv-contrib-python packages installed in your Python environment to
run this code. You can install them using pip:

pip install pyrealsense2
pip install opencv-contrib-python

Please note that accurate camera calibration is crucial for the pose estimation to be correct. 
The calibration process involves capturing images of a known pattern (like a checkerboard) from different angles 
and using them to compute the camera parameters. OpenCV provides functions such as cv2.calibrateCamera for this purpose. If you haven’t done this yet, you should calibrate your camera before attempting to estimate the pose of the ArUco board.
'''
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

# Initialize the camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Load the predefined dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)

# Create parameters to detect markers with tuned settings
parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshWinSizeMin = 5
parameters.adaptiveThreshWinSizeMax = 20
parameters.adaptiveThreshWinSizeStep = 5
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 3
parameters.cornerRefinementMinAccuracy = 0.01
parameters.cornerRefinementMaxIterations = 50

# Camera calibration parameters (replace with your actual calibration results)
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])
dist_coeffs = np.array([k1, k2, p1, p2, k3])

# Define the ArUco board parameters
markersX = 3
markersY = 2
markerLength = 120  # Marker length in millimeters
markerSeparation = 10  # Marker separation in millimeters
board = aruco.GridBoard_create(markersX, markersY, float(markerLength) / 1000,
                               float(markerSeparation) / 1000, dictionary)

# Align the depth frame to the color frame
align = rs.align(rs.stream.color)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Detect markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, dictionary, parameters=parameters)

        # Refine detected markers
        aruco.refineDetectedMarkers(color_image, board, corners, ids, rejectedImgPoints)

        # Estimate the posture of the board, which requires a set of detected markers
        retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None)

        # If the pose is correctly estimated
        if retval:
            # Draw the markers and axis on the image
            aruco.drawDetectedMarkers(color_image, corners, ids)
            aruco.drawAxis(color_image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

            # Print the pose of the board
            print("Rotation Vector:\n", rvec)
            print("Translation Vector:\n", tvec)

        # Display the resulting frame
        cv2.imshow('Frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
