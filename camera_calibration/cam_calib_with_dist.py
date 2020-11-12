import numpy as np
import cv2
import glob# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)def calibrate(dirpath, prefix, image_format, square_size, width=9, height=6):
""" Apply camera calibration operation for images in the given directory path. """
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
objp = np.zeros((height*width, 3), np.float32)
objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

objp = objp * square_size # if square_size is 1.5 centimeters, it would be better to write it as 0.015 meters. Meter is a better metric because most of the time we are working on meter level projects.

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.# Some people will add "/" character to the end. It may brake the code so I wrote a check.
if dirpath[-1:] == '/':
    dirpath = dirpath[:-1]images = glob.glob(dirpath+'/' + prefix + '*.' + image_format) #

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (width, height), None)    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

return [ret, mtx, dist, rvecs, tvecs]