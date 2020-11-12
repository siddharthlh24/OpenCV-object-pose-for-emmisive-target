
import cv2
import numpy as np


def drawrect( img , pt1):                              # if you got rectangular LEDs
    drec = 10
    cv2.rectangle(img, pt1=(pt1[0]-drec,pt1[1]-drec), pt2=(pt1[0]+drec,pt1[1]+drec), color=(0,0,255), thickness=-1)

color = (0,0,255)
  
# Line thickness of 2 px
thickness = -1
radius = 15

image = 255*np.ones(shape=[500, 500,3], dtype=np.uint8)
image = cv2.circle(image, (50,50), radius, color, thickness)
image = cv2.circle(image, (50,450), radius, color, thickness)
image = cv2.circle(image, (450,50), radius, color, thickness)
image = cv2.circle(image, (450,450), radius, color, thickness)
image = cv2.circle(image, (350,150), radius, color, thickness)
#image = cv2.circle(image, (75,300), radius, color, thickness)

"""drawrect(image,(256,256))
drawrect(image,(256,256-100))
drawrect(image,(256,256+100))
drawrect(image,(256-200,256))
drawrect(image,(256+200,256))"""

"""for v in range(100):
    image = cv2.circle(image, (rd.randint(0,640),rd.randint(0,480)), rd.randint(1,3), color, thickness)"""
cv2.imwrite("tracking_marker.png", image)
cv2.imshow("White Blank", image)
cv2.waitKey(0)