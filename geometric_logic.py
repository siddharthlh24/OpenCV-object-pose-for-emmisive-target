import cv2
import cv2 as cv
from matplotlib.pyplot import close
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

#################################################################################
font = cv.FONT_HERSHEY_SIMPLEX 
org = (10,400) 
fontScale = 0.7
color = (255,0,0)
thickness = 2
#################################################################################
def e_dist(a, b, metric='euclidean'):

    """Distance calculation for 1D, 2D and 3D points using einsum
    : a, b   - list, tuple, array in 1,2 or 3D form
    : metric - euclidean ('e','eu'...), sqeuclidean ('s','sq'...),
    :-----------------------------------------------------------------------
    """
    a = np.asarray(a)
    b = np.atleast_2d(b)
    a_dim = a.ndim
    b_dim = b.ndim
    if a_dim == 1:
        a = a.reshape(1, 1, a.shape[0])
    if a_dim >= 2:
        a = a.reshape(np.prod(a.shape[:-1]), 1, a.shape[-1])
    if b_dim > 2:
        b = b.reshape(np.prod(b.shape[:-1]), b.shape[-1])
    diff = a - b
    dist_arr = np.einsum('ijk,ijk->ij', diff, diff)
    if metric[:1] == 'e':
        dist_arr = np.sqrt(dist_arr)
    dist_arr = np.squeeze(dist_arr)
    return dist_arr
#################################################################################
def Diff(li1, li2):
    li_dif = [i for i in li1 + li2 if i not in li1 or i not in li2]
    return li_dif
################################################################################
img = cv2.imread('black_mark.png')
img = cv2.rotate(img, cv2.ROTATE_180) 
hsv =cv.cvtColor(cv.blur(img,(7,7)),cv.COLOR_BGR2HSV)
#hsv =cv.cvtColor(img,cv.COLOR_BGR2HSV)

mask1 = cv2.inRange(hsv, (0,100,80), (20,255,255))
mask2 = cv2.inRange(hsv, (165,100,80), (180,255,255))

## Merge the mask and crop the red regions
mask = cv2.bitwise_or(mask1, mask2 )
#kernel = np.ones((30,30),np.uint8)
kernel = cv2.cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))

#mask = cv.dilate(mask,kernel,iterations = 1)
#mask = cv.erode(mask,kernel,iterations = 1)
#ret2,th2 = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)
cv2.imshow('mask',mask)

contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
img = cv2.drawContours(img, contours, -1, (0,255,0), 3)

list_points = []

for c in contours:
    M = cv2.moments(c)
    if(cv2.contourArea(c) > 50 and cv2.contourArea(c) < (300*300)):
        #print('ok')
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        list_points.append(np.array([cx,cy]))
        plt.plot(cx,cy,'bo')

list_points = np.asarray(list_points)
print(list_points)

if(len(list_points)==5):
        list_points=np.array(list_points)

        hull = np.squeeze(cv.convexHull(list_points))     # convex hull/ reject inner out of plane point
        print(list_points)
        print(hull)
        centroid = hull.mean(axis=0)

        idx = np.argsort(e_dist(list_points, (centroid[0],centroid[1])))
        close = list_points[idx]
        in_pt = close[0]
    
        cxx,cyy = in_pt[0],in_pt[1]
        print('inpt',cxx,cyy)

        idx = np.argsort(e_dist(hull, (cxx,cyy)))
        closest = hull[idx]

        tanvals = []
        for k in hull:
            xp,yp = k[0],k[1]
            tanvals.append(np.arctan2((yp-cyy),(xp-cxx)))
        tanvals = np.asarray(tanvals)
        
        print('hull',hull)
        sorted_hull = hull[tanvals.argsort()]
        print('srt_hull',sorted_hull)

        print('tanvals',tanvals)
        print('closest',closest[0])
        index = np.where(tanvals==np.arctan2((closest[0][1]-cyy),(closest[0][0]-cxx)))[0][0]
        print('index',index)

        sorted_hull=np.roll(sorted_hull,index,axis=0)

        pt1,pt2,pt3,pt4 = sorted_hull
        cv.putText(img, str("pt1"), tuple(pt1.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
        cv.putText(img, str("pt2"), tuple(pt2.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
        cv.putText(img, str("pt3"), tuple(pt3.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
        cv.putText(img, str("pt4"), tuple(pt4.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
        cv.putText(img, str("in_pt"), tuple(in_pt.ravel()), font, fontScale, color, thickness, cv.LINE_AA)


cv2.imshow('img',img)
#plt.show()
cv2.waitKey(0)
