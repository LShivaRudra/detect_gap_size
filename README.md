## Aim of the Repo
To detect the gap in front of the drone using a camera and to predict if the drone can pass through it or not. If not, the amount with which it has to morph should be predicted.

## Repo Structure:
- **src**: contains all the well commented scripts
- **test_results**: contains videos, pictures, etc. of results 
- **README.md**

---

## Test 1
This test aims at the following:
- A Region Of Interest(**ROI**) is selected and is used on every image frame the camera captures. The dimensions of this ROI are calculated as follows:
	- Assume the drone in front of the camera at a distance of 1m. Now, whatever the drone's dimensions will be in the image frame will be used as the dimensions of the ROI(here taken as a rectangle). 
- Now, if any obstacle lies at 1m and is within the ROI, there cannot be any box that can be drawn within the ROI through which the drone can pass through. Now, the amount with which the drone has to bend to pass through can be estimated.
- If there are obstacles which are present at a distance more than 1m, based on the depth values from the depth camera, the largest rectangle within the ROI of the image is estimated and comparing the dimensions of that rectangle in the 3D world, it is to be predicted if the drone can pass through or not without morphing. Also, the amount with which it has to morph can be predicted.

### Assumptions of Test 1:
- There are no obstacles within the set threshold(here taken as 1m) in front of the drone.
- The drone only moves along the z-direction of the depth camera.

### Dimensions of the Drone used in the code:
- 0.6x0.45x0.6 (all in meters)

### Sensor used
- Intel Realsense D455 depth camera

### Methodology followed in the code
- The depth, color images from the D455 camera are stored.
- With the depth image, all the pixels which are closer than a threshold(1m) to the camera are masked(turned 0).
-  Then, all the pixels within the ROI with non-zero intensities are made to retain their value and the rest pixel intensities are made 0.
-  Contours are drawn to the regions which have non-zero pixel intensity values.
-  With the contours drawn, the bounding rectangles are drawn within the contours and the rectangle with maximum are is found out among them. This is the rectangle which will be used to find if the drone can pass through the gap without morphing.
-  Assuming that all the points of the maximum area rectangle have same depth from the camera, the dimensions of the rectangle are found out in the 3D world and are compared with that of the drone.

### Equations used
- u = int(x * fx / distance)
v = int(y * fy / distance)
where (u,v) are the coordinates of any pixel in image frame and distance is the depth value at (u,v) measured by the camera.

### Parameters used
- Camera intrinsic parameters:
	- fx: Focal length along x = 6.0970550296798035e+02
	- fy: Focal length along y = 6.0909579671294716e+02
	- cx: x coordinate of the center of the image = 320
	- cy: y coordinate of the center of the image = 240

### Links/References used:
- [D455-Opencv](https://dev.intelrealsense.com/docs/opencv-wrapper)

---

## Test 2
This test aims at the following:
- A Region Of Interest(**ROI**) is selected but unlike 'Test 1', the dimensions of ROI are not calculated using the drone's dimensions here. Limits are:
	-	**Dims(Drone in image frame) < Dims(ROI) < Dims(Image)**
	-	Here  **Dims(Drone in image frame)** is calculated by projecting the drones dimensions in 3D onto the image frame if the drone is 2m in front of the camera
- Now another image is constructed from the color image taking only the ROI and setting the intensities of the remaining pixels as 0. A mask called 'mask_bg' (which is used to retain the intensities of pixels with depth greater than setting the rest to 0) is applied on the ROI to find out the freespace farther than 2m. 
- In this new masked ROI image, the contour with maximum area is found out and a bounding rectangle is drawn representing that gap.
- Conditions:
	- if the dimensions of the bounding rectangle are closer(a tolerance value of 50 pixels is used) to that of the ROI, it means that the drone is free to move forward without checking any other conditions for the current image frame.
	- else if the bounding box is smaller than the ROI:
		- if the centers of box and ROI are aligning(this is checked using a tolerance value of 50 pixels), the dimensions of both are compared to check if the drone can pass through without morphing.
		- else if the centers are not aligning, conditions are checked and drone is given instructions: turn left/right/upwards/downwards.

### Dimensions of the Drone used in the code:
- 0.3x0.15x0.3 (all in meters)

### Parameters used
- Camera intrinsic parameters:
	- fx: Focal length along x = 610
	- fy: Focal length along y = 610
	- cx: x coordinate of the center of the image = 320
	- cy: y coordinate of the center of the image = 240

### Parameters to be tuned:
- Dimensions of the bounding box: roi_im_b, roi_im_h
- Tolerances: tol, tol_to_align_centers

### Issues:
- The bounding box obtained from the maximum area contour is not that accurate. So this can lead to unreliable results.

### Improvements for the next test:
- Rather than just taking the maximum area contour, to select the contour which has weightage for pixels with higher depth from the camera also. This can solve the problem in the test2 video where wall(which occupied higher area in the image frame) was shown as better path instead of the open space behing the door of the room(which had lower area in the image frame but higher depth values).

### Links/References used:
- [Paper: GapFlyt](http://prg.cs.umd.edu/research/gapflyt_files/GapFlyt-RAL2018.pdf)

---
