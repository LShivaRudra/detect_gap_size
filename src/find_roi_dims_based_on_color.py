import cv2
import numpy as np
import pyrealsense2 as rs

# Initialize the camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

image_width = 640
image_height = 480
tol_pix = 5
tol_d = 0.02
fx, fy, cx, cy = 6.0970550296798035e+02, 6.0909579671294716e+02, 3.1916667152289227e+02, 2.3558360480225772e+02

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Get the color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert the color frame to a NumPy array
        frame = np.asanyarray(color_frame.get_data())

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the orange color
        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([30, 255, 255])

        # Threshold the image to find the orange regions
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the contour with the maximum area
        max_contour = max(contours, key=cv2.contourArea, default=None)

        # Draw bounding box around the largest contour
        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Get the centroid of the bounding box
            centroid = (int(x + w/2), int(y + h/2))

            # Check if the depth frame is valid at the centroid
            if 0 <= centroid[0] < image_width and 0 <= centroid[1] < image_height:
                # Get the depth value at the centroid
                depth_centroid = depth_frame.get_distance(centroid[0], centroid[1])

                # Check additional conditions for the ROI
                if (
                    abs(centroid[0] - int(image_width/2)) < tol_pix and
                    abs(centroid[1] - int(image_height/2)) < tol_pix and
                    abs(depth_centroid - 2) < tol_d
                ):
                    print("Dims of ROI in image frame: ", w, " and ", h)
                    roi_width_3d = (w * depth_centroid) / fx
                    roi_height_3d = (h * depth_centroid) / fy
                    print("Dims of ROI 3d: ", roi_width_3d, " and ", roi_height_3d)
                    print(" ")

        # Display the original frame and the result
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Orange Regions', mask)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming and close all OpenCV windows
    pipeline.stop()
    cv2.destroyAllWindows()
