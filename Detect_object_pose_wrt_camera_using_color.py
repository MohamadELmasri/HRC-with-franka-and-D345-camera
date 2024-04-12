import cv2
import numpy as np
import pyrealsense2 as rs

'''
In this code:

The color stream is enabled alongside the depth stream.
A color range is defined for the object you want to detect. You’ll need to replace lower_color and 
upper_color with the actual BGR color range of your object.
The largest contour within the color range is found, and its center is calculated.
The depth value at the center of the contour is retrieved from the depth image.
The contour and its center are drawn on the color image, which is then displayed.
Please note that the color range for detection and the method of finding the object’s pose 
may vary depending on the specific requirements of your application and the characteristics of the 
object you’re trying to detect. You may need to adjust the color range and the contour processing accordingly. 
Additionally, ensure that the lighting conditions are consistent for reliable color detection.
'''
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Define the range of the color you want to detect
        # Replace with the actual color range of your object
        lower_color = np.array([30, 150, 50])
        upper_color = np.array([255, 255, 180])

        # Threshold the color image to get only the desired color
        mask = cv2.inRange(color_image, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        areas = [cv2.contourArea(c) for c in contours]
        if areas:
            max_index = np.argmax(areas)
            cnt = contours[max_index]

            # Get the moments to calculate the center of the contour
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # Get the depth value from the depth image
                depth = depth_image[cy, cx]
                print(f"Detected object at coordinates (X: {cx}, Y: {cy}, Depth: {depth})")

                # Draw the contour and center of the shape on the image
                cv2.drawContours(color_image, [cnt], -1, (0, 255, 0), 3)
                cv2.circle(color_image, (cx, cy), 7, (255, 255, 255), -1)

        # Display the resulting frame
        cv2.imshow('Detected Object', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
