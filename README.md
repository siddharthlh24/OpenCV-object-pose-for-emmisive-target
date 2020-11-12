# OpenCV-object-pose-for-emmisive-target
This project is for people who are looking for object pose estimation/augmented reality solutions where reflective fiducial markers are not an option. This method is designed to work for emmisive target elements, such as LEDs or coloured bulbs. <br>

# Steps required for pose estimation:
1) Obtain and calibrate a camera system.<br>
2) Create a trackable pattern and store 3d information of the pattern using the physical construction information of the target.<br>
3) Use camera to track key points and establish a correspondence between the points being tracked and 3D reference points.<br>
4) Relative position and Orientation (Pose) are obtained by solving the Perspective-N-Point problem. <br>

# Augmented reality o/p
![](media/ezgif-3-6bc187371c65.gif)

# Object Pose Reprojection
![](media/ezgif-3-7cfb867c5025.gif)

# Step 1 : Calibrating a camera
Any camera has physical properties of how they represent a point in 3D space in 2D image space. Camera calibration is the process of estimating intrinsic and/or
extrinsic parameters. Intrinsic parameters deal with the camera's internal characteristics, such as, its focal length, skew, distortion, and image centre.The camera will also have some inherent defects,such as barrel or pincushion distortion etc. which are represented in a distortion matrix. We use several images of chessboard to calibrate camera ( zheng method ). ( Print the chessboard pattern on paper and measure the side of a chessboard square to input into the programme )<br>

![](media/template.JPG)
