import cv2
import numpy as np
import pyrealsense2 as rs

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Camera Intrinsics
fx, fy, cx, cy = 610, 610, 320, 240
image_width = 640
image_height = 480

# Drone Dimensions(in meters)
drone_width = 0.3
drone_height = 0.15
drone_dim_top_view = 0.3 #drone's third dimension

# These are the dimensions of the ROI(rectangle) where the img processing will be done. Limits are: min: drone's dims in image frame; max: image dims
roi_im_b = image_width - 100 #the bredth of the ROI
roi_im_h = image_height - 100 #the height of the ROI

# Drone's dims in image frame assuming that it is present 2m in front of the camera
drone_im_b = 92 # bredth 
drone_im_h = 46 # height 

try:
    while True:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame() 
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        if not depth_frame or not color_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_HSV)

        # Define near threshold (adjust as needed)
        far_threshold = 2000 + 100 
        near_threshold = 2000 - 100

        # Coordinates of the ROI rectangle box in image frame
        roi_x1, roi_y1 = int(cx - roi_im_b/2), int(cy - roi_im_h/2)
        roi_x2, roi_y2 = int(cx + roi_im_b/2), int(cy + roi_im_h/2)

        # Out of the color image, select only the ROI pixels setting the intensity of the rest to 0
        roi_image = np.zeros_like(color_image)
        roi_image[roi_y1:roi_y2, roi_x1:roi_x2] = color_image[roi_y1:roi_y2, roi_x1:roi_x2]

        mask_bg = np.where((depth_image > far_threshold), 255, 0).astype(np.uint8) # Binary Mask for the background pixels(depth greater than 2.1 meters)
        mask_fg = np.where((depth_image < near_threshold), 255, 0).astype(np.uint8) # Binary Mask for the foreground pixels(depth less than 1.9 meters)
        mask_uncertain = np.where((depth_image > near_threshold) & (depth_image < far_threshold), 255, 0).astype(np.uint8) # Binary Mask for the uncertain pixels(depth lies between 1.9 meters and 2.1 meters)

        # Apply the mask to the grayscale image using mask_bg since we are interested to pass through the gap
        bg_info_image = cv2.bitwise_and(roi_image, roi_image, mask=mask_bg)

        bg_info_gray = cv2.cvtColor(bg_info_image, cv2.COLOR_BGR2GRAY) # Convert to gray scale before finding the contours

        contours, _ = cv2.findContours(bg_info_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0 # To store the area of rectangle with maximum area in the gap within ROI
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
        cx_gap_box = int(x+w/2)
        cy_gap_box = int(y+h/2)
        tol_to_align_centers = 50 # This tolerance is used to check if the center of the max_area rectangle is aligning with that of ROI

        tol = 50 # This tolerance is used to check if there is any obstacle in front of the drone within 2 m. If that space is clear, the drone can pass through without checking for the remaining conditions for the current frame
        
        if(abs(w-roi_im_b)<tol and abs(h-roi_im_h)<tol): # Condition to check if dimensions of max_area rectangle are close to ROI in the current img frame
            print("It is a freespace ahead! The drone can pass through freely")

        elif(abs(cx_gap_box-cx)<tol_to_align_centers and abs(cy_gap_box-cy)<tol_to_align_centers): # centers are aligning
            if(w>drone_im_b and h>drone_im_h):
                print("The drone can pass safely without morphing") # If gap dimensions are more than that of the drone, move forward
            else:
                print("The drone has to morph") # If gap dimensions are less than that of the drone, morph

        elif(abs(cx_gap_box-cx)>tol_to_align_centers and abs(cy_gap_box-cy)<tol_to_align_centers): # x-coordinate of the centers are not aligning
            if((cx_gap_box-cx)<0):
                print("Try turning the drone towards left to check for better window to pass through")
            
            elif((cx_gap_box-cx)>0):
                print("Try turning the drone towards right to check for better window to pass through")

        elif(abs(cx_gap_box-cx)<tol_to_align_centers and abs(cy_gap_box-cy)>tol_to_align_centers): # y-coordinate of the centers are not aligning
            if((cy_gap_box-cy)<0):
                print("Try turning the drone upwards to check for better window to pass through")
            
            elif((cy_gap_box-cy)>0):
                print("Try turning the drone downwards to check for better window to pass through")

        cv2.rectangle(bg_info_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 255), 2) # Draw rectangle representing the ROI
        cv2.rectangle(bg_info_image, (x, y), (x+w, y+h), (0, 255, 0), 2) # Draw rectangle representing the max_area box 

        cv2.imshow('Original Color', color_image)
        cv2.imshow('Image of interest', bg_info_image)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
