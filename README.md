---
title: Gap Size Detection
updated: 2023-12-09 22:58:34Z
created: 2023-12-05 20:45:14Z
latitude: 28.66189760
longitude: 77.22739580
altitude: 0.0000
---

## Aim of the Repo
To detect the gap in front of the drone using a camera and to predict if the drone can pass through it or not. If not, the amount with which it has to morph should be predicted.

## Repo Structure:
- **src**: contains all the well commented scripts
- **test_results**: contains videos, pictures, etc. of results 
- **README.md**

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
