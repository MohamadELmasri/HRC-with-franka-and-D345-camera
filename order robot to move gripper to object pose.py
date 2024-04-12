import rospy
import pyrealsense2 as rs
from panda_robot import PandaArm
import numpy as np

'''
This script initializes the RealSense camera, waits for the color and depth frames, and assumes that the object detection
and pose estimation are already implemented. The object_pose dictionary should be replaced with the actual 
pose data relative to the robot’s base frame, which you would obtain from your object detection algorithm.

Please ensure that you have the correct transformations from the camera frame to the robot base frame to accurately
position the gripper based on the camera’s data. The code also assumes that the panda_robot ROS package is set up
correctly for controlling the Panda robot.
'''

# Initialize the ROS node
rospy.init_node('panda_move_to_detected_object')

# Create an instance of the PandaArm class
panda = PandaArm()

# Configure the Intel RealSense D435i
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the camera stream
pipeline.start(config)

try:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        raise RuntimeError('Could not acquire depth or color frames.')

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Assume object detection and pose estimation is done here
    # and you have the object_pose in the robot base frame
    object_pose = {'position': object_position, 'orientation': object_orientation}  # Replace with actual data

    # Move the gripper to the object pose
    panda.move_to_cartesian_pose(position=object_pose['position'], orientation=object_pose['orientation'])

    # Open the gripper (if needed)
    panda.get_gripper().open()

    # Perform any additional actions here

finally:
    # Stop the camera pipeline
    pipeline.stop()

    # Shutdown the ROS node
    rospy.signal_shutdown('Finished moving gripper to detected object pose')
