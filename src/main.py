import cv2
import numpy as np
import pyrealsense2 as rs
import math

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Camera Intrinsics
fx, fy, cx, cy = 6.0970550296798035e+02, 6.0909579671294716e+02, 3.1916667152289227e+02, 2.3558360480225772e+02

# Drone Dimensions(in meters)
drone_width = 0.6
drone_height = 0.5
drone_dim_top_view = 0.6 #drone's third dimension

# The dimensions of the bounding box. This bounding box is taken in such a way that if the drone is present in front of camera at a distance of 1m(where we are checking if there is any gap or not), it can pass through this bounding box
roi_im_b = 365 #the bredth of the ROI(rectangle)
roi_im_h = 274 #the height of the ROI

try:
    while True:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame() 
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Define near threshold (adjust as needed)
        near_threshold = 1000 + drone_dim_top_view*1000 # This value is in millimeters. Here drone_dim_top_view is added so as to make sure that the drone can completely pass through the gap

        # Create binary mask for pixels at a depth less that near_threshold 
        near_mask = np.where(depth_image < near_threshold, 0, 255).astype(np.uint8)

        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY) # Computed to reduce the number of channels and save computation

        # Apply the mask to the grayscale image
        near_colored = cv2.bitwise_and(gray_image, gray_image, mask=near_mask)

        # Draw ROI 
        cv2.rectangle(near_colored, (int(cx - roi_im_b/2), int(cy - roi_im_h/2)), (int(cx + roi_im_b/2), int(cy + roi_im_h/2)), (255, 0, 0), 2)

        # Find out the top-left and bottom-right coordinates of the ROI
        roi_x1, roi_y1 = int(cx - roi_im_b/2), int(cy - roi_im_h/2)
        roi_x2, roi_y2 = int(cx + roi_im_b/2), int(cy + roi_im_h/2)

        # Create a mask which is used to take only the non-zero pixel values within the ROI 
        near_mask_within_roi = np.zeros_like(near_mask)
        near_mask_within_roi[roi_y1:roi_y2, roi_x1:roi_x2] = np.where(near_mask[roi_y1:roi_y2, roi_x1:roi_x2] > 0, 255, 0).astype(np.uint8)

        # Apply the near_mask_within_roi to the near_colored image
        near_colored_roi = cv2.bitwise_and(near_colored, near_colored, mask=near_mask_within_roi)

        # Find out the contours within the non-zero region in the near_colored_roi
        contours, _ = cv2.findContours(near_mask_within_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0 # To store the area of rectangle with maximum area in the non-zero region in the near_colored_roi
        max_rect = None # Used to store coordinates of the max-area rectangle. It's through this rectangle which we want the drone to pass

        for contour in contours: # Iterate over all the contours
            # Get the bounding rectangle
            x, y, w, h = cv2.boundingRect(contour) 

            # Calculate the area of the rectangle
            rect_area = w * h

            # Update max_area and max_rect if the current rectangle has a larger area
            if rect_area > max_area:
                max_area = rect_area
                max_rect = (x, y, w, h)

        # Draw the maximum rectangle on the image
        x, y, w, h = max_rect

        # Coordinates of midpoints of the maximum rectangle in 3D
        z1 = depth_frame.get_distance(x,int(y+h/2))
        z2 = depth_frame.get_distance(x+w,int(y+h/2))
        z3 = depth_frame.get_distance(int(x+w/2),y)
        z4 = depth_frame.get_distance(int(x+w/2),y+h)

        x1_3d, y1_3d = x*z1/fx, int(y+h/2)*z1/fy
        x2_3d, y2_3d = (x+w)*z2/fx, int(y+h/2)*z2/fy
        x3_3d, y3_3d = int(x+w/2)*z3/fx, y*z3/fy
        x4_3d, y4_3d = int(x+w/2)*z4/fx, (y+h)*z4/fy

        # Assumption: We assume that all the points in the hole/gap through which the drone is to pass lie on the same plane(z1,z2,z3 and z4 should be approximately same)
        gap_width = abs(x2_3d - x1_3d)
        gap_height = abs(y4_3d - y3_3d)
        
        print("The Gap dimensions as width x height are: ",gap_width," x ",gap_height)
        if gap_width>drone_width and gap_height>drone_height:
            print("The drone can pass through the gap")
        else:
            print("The drone cannot pass through the gap")

        cv2.rectangle(near_colored_roi, (x, y), (x + w, y + h), (255, 255, 255), 2)

        # cv2.imshow('Original Color', color_image)
        cv2.imshow('Depth Image', depth_image)
        cv2.imshow('Near Masked Image', near_colored)
        cv2.imshow('Near Masked ROI Image(Result)', near_colored_roi)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
