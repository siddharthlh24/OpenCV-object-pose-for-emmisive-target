# OpenCV-object-pose-for-emissive-target
This project is for people who are looking for object pose estimation/augmented reality solutions where reflective fiducial markers are not an option. This method is designed to work for emissive target elements, such as LEDs or colored bulbs. I am using red circles to emulate red LEDs since I don't have physical LEDs available on hand. Suffice to say, both will function in a similar manner. This also uses only an RGB monocular camera.  
Demo video: https://youtu.be/RGoBDAEkDBU  

# Steps required for pose estimation:
1) Obtain and calibrate a camera system.  
2) Create a trackable pattern and store 3D information of the pattern using the physical construction information of the target.  
3) Use the camera to track key points and establish a correspondence between the points being tracked and 3D reference points.  
4) Relative position and orientation (Pose) are obtained by solving the Perspective-N-Point problem.  
5) Output filtering (using moving averages / low pass filter to reduce jitter).

# Augmented Reality Output
![](media/ezgif-3-6bc187371c65.gif)

# Object Pose Reprojection
![](media/ezgif-3-7cfb867c5025.gif)

# Step 1: Calibrating a camera
Any camera has physical properties of how they represent a point in 3D space in 2D image space. Camera calibration is the process of estimating intrinsic and/or extrinsic parameters. Intrinsic parameters deal with the camera's internal characteristics, such as its focal length, skew, distortion, and image center. The camera will also have some inherent defects, such as barrel or pincushion distortion, which are represented in a distortion matrix. We use several images of a chessboard to calibrate the camera (Zheng method).  
![](media/template.JPG)  
a) Print the [required chessboard pattern](camera_calibration/calib_pattern.png) on paper and measure the side of a chessboard square to input into the program.  
b) The calibration matrices for the given camera save as a **.npz** archive, which is used during tracking and pose estimation. I have included one as an example (poco_x2.npz) and it can be used if we use any 1080p stream from phone cameras.

# Step 2 + 3: Creating trackable pattern and tracking system for LEDs
This is the most tricky part. We will need to create an LED pattern where each LED can be individually detected by using special geometric rules.  
**Why do we need to do this?**  
Because there is **absolutely no way** to establish a correspondence between the LEDs detected in the image and the one in the created pattern **without geometric rules**. I have provided one example here and have included programs to [prototype new patterns](bespoke_led_pattern_gen.py) and develop [geometry rules](geometric_logic.py) for them. It is important to take care while developing patterns, not to create ambiguities and to minimize iterative detection methods.  
**In the picture below, on the left is the generated tracking pattern, and on the right is the geometric logic to detect each LED.**

![](media/tracking_marker.png)  
a) Detect all LEDs using a color space change to HSV and threshold for red. Find the centers for contours and store them in an array.  
b) Calculate a **convex hull** to detect the outer 4 points/LEDs. These will form **pt1, pt2, pt3, pt4**, but we **don't know** in which order yet.  
c) The remaining point/LED not part of the convex hull will be **in_pt**. Alternatively, we can find the center of the convex hull, and in_pt will always be the nearest point (green rectangle).  
d) The point closest to in_pt is taken as pt1 (blue rectangle). Then we go **clockwise** and assign other points on the hull in order. This is done by **calculating slopes and barrel shifting** the points according to the values.  
e) We have now identified every LED.

# Step 4: Perspective-n-point 
Given a set of n 3D points in a world reference frame and their corresponding 2D image projections, as well as the calibrated intrinsic camera parameters, we need to determine the 6 DOF pose of the camera in the form of its rotation and translation with respect to the world/target. An inverse of the rotation matrix will, in effect, provide the target pose. This follows the perspective projection model for pinhole cameras:  
![](media/pnp_desc.JPG)


```
_, rvecs, tvecs, inliers = cv.solvePnPRansac(objpts, dst, mtx, dist,cv.SOLVEPNP_IPPE)
```
The above function will return rotation and translation vectors which can be further processed and used to project any 3D object. The pattern used here is coplanar, however, that is not necessary. Using non-coplanar 3D points helps to reduce ambiguities and measurement inaccuracies.  
Refer to this link: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga357634492a94efe8858d0ce1509da869 for different variants of PNP.  

```
SOLVEPNP_ITERATIVE 
SOLVEPNP_EPNP 
SOLVEPNP_P3P 
SOLVEPNP_DLS 
SOLVEPNP_UPNP 
SOLVEPNP_AP3P 
SOLVEPNP_IPPE 
SOLVEPNP_IPPE_SQUARE 
```
The current implementation also includes 2 moving averages for rotation and translation respectively to smooth out jitter and increase accuracy. However, note that higher sizes for the moving average buffer will make response to change slower. A value of 3 to 5 is recommended for rotation and 1 for translation (30 fps camera).  

Object back projection is done by the following function:  
```
imgpts, jac = cv.projectPoints(cube3d, rvecs, tvecs, mtx, dist)
```
# Calculating derivatives
As long as the time between frame capture is constant, rotation and linear velocities can be estimated easily.
