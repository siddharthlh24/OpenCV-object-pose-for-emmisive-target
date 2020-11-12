# OpenCV-object-pose-for-emmisive-target
This project is for people who are looking for object pose estimation/augmented reality solutions where reflective fiducial markers are not an option. This method is designed to work for emmisive target elements, such as LEDs or coloured bulbs. <br>

# Steps required for pose estimation:
1) Obtain and calibrate a camera system.<br>
2) Create a trackable pattern and store 3d information of the pattern using the physical construction information of the target.<br>
3) Use camera to track key points and establish a correspondence between the points being tracked and 3D reference points.<br>
4) Relative position and Orientation (Pose) are obtained by solving the Perspective-N-Point problem. <br>
