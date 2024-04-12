import pyrealsense2 as rs
import numpy as np
import cv2.aruco as aruco
import cv2
import math
import pandas as pd 
import time 

# Useful Functions 
# Go from T matrix to rotation matrix r and translation vector t 
def rt_from_matrix(M):
    r= M[0:3, 0:3] 
    t=  M[0:3, 3]
    return (r, t)

# Check if a matrix is a valid rotation matrix
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    if n>1e-3:
        print("Error! Not a rotation matrix")
    return n<1e-3

# Transform rotation matrix to euler angles: same as MATLAB
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    x=math.degrees(x)
    y=math.degrees(y)
    z=math.degrees(z)
    return np.array([z, y, x])

# Configure depth stream
pipeline = rs.pipeline()
config = rs.config()

# Configure resolution and frame rate of depth sensor
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

# Information to be recorded 
N = 1000
pose = np.zeros((N,6))
timeMeas = np.zeros((N,1)) 
stand_dev = np.zeros((N,1)) 
k = 0 # counter 

# Start timer 
start = time.time()
recordingTime = 5 # in seconds 
print("Recording...")

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Get depth frame
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Detection of ArUco Board
        arucoParams = aruco.DetectorParameters_create()

        # Board's Parameters 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        markersX = 3
        markersY = 2
        markerLength = 120 # in mm 
        markerSeparation = 10 # in mm 
        board = aruco.GridBoard_create(markersX, markersY, float(markerLength),
                                                    float(markerSeparation), aruco_dict)

        # Detection: we start by detecting all markers 
        gray_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray_image, aruco_dict, parameters=arucoParams)  # First, detect markers
        aruco.refineDetectedMarkers(gray_image, board, corners, ids, rejectedImgPoints)

        if len(corners) > 0: # if there is at least one marker detected
            im_with_aruco_board = aruco.drawDetectedMarkers(depth_image, corners, ids)
            markNum, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, None, None, rvec=np.float32([0,0,0]), tvec=np.float32([0,0,0]))  
            
            if markNum != 0:
                # Draw pose 
                im_with_aruco_board = cv2.drawFrameAxes(im_with_aruco_board, None, None, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement

                # Now, you have a board frame with the following transformation wrt camera frame 
                (R_board2cam, _) = cv2.Rodrigues(rvec) # go to rotation matrix representation 
                
                # Get Euler Angles of board frame wrt camera frame
                EulerAng = rotationMatrixToEulerAngles(R_board2cam)

                # Record pose and time 
                timeMeas[k][0] = time.time() - start
                pose[k][0] = tvec[0]
                pose[k][1] = tvec[1]
                pose[k][2] = tvec[2]
                pose[k][3] = EulerAng[0]
                pose[k][4] = EulerAng[1]
                pose[k][5] = EulerAng[2]

                # Increase counter 
                k += 1

                # Stopping condition 
                if time.time() - start > recordingTime or k == N:
                    stand_dev[0][0] = np.std(pose[0:k, 0])
                    stand_dev[1][0] = np.std(pose[0:k, 1])
                    stand_dev[2][0] = np.std(pose[0:k, 2])
                    stand_dev[3][0] = np.std(pose[0:k, 3])
                    stand_dev[4][0] = np.std(pose[0:k, 4])
                    stand_dev[5][0] = np.std(pose[0:k, 5])
                    array = np.hstack((timeMeas, pose, stand_dev))
                    df = pd.DataFrame(array, columns=['Time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'std:x', 'std:y', 'std:z', 'std:roll', 'std:pitch', 'std:yaw'])
                    filepath = 'measPose_aruco_board.xlsx'
                    df.to_excel(filepath, sheet_name='Sheet1', index=False)
                    print("Recording is finished!")
                    break

        else:
            im_with_aruco_board = depth_image

        # Display board 
        cv2.imshow("ArUco Board", im_with_aruco_board)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    # Stop streaming
    pipeline.stop()
