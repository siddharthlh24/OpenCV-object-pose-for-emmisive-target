from os import scandir
import cv2
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
import time
import socket
from imutils.video import VideoStream
UDP_IP_ADDRESS = "192.168.8.102"
UDP_PORT_NO = 11111
clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

##############################################################################
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
font = cv.FONT_HERSHEY_SIMPLEX 
org = (10,400) 
fontScale = 0.7
color = (255,0,0)
thickness = 2
#################################################################################
with np.load('poco_x2.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
###############################################################################
objpts=np.float32([[-6,6,0],[6 ,6,0],[6,-6,0],[-6,-6,0],[4.5,-4.5,0]])                      #real world 3d object keypoints, [ this will be position of LEDs/ red circles ]
###############################################################################
cube3d = np.float32([[-8,-8,0],[-8,8,0], [8,8,0], [8,-8,0],[-3,0,-10],[0,3,-10],[3,0,-10],[0,-3,-10],[0,0,0] ]).reshape(-1,3)         #3d object for augmented reality projection, can be anything
#############################
def draw(img, imgpts): 

    img = cv2.polylines(img, [np.int32(imgpts[0:4])],  
                      True, (0,0,0), 5)
    
    cv.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[2].ravel()), (0,0,0), 5)
    cv.line(img, tuple(imgpts[1].ravel()), tuple(imgpts[3].ravel()), (0,0,0), 5)

    cv.line(img, tuple(imgpts[0].ravel()), tuple(imgpts[4].ravel()), (0,0,0), 5)                        #draw function for augmented reality 2d projection
    cv.line(img, tuple(imgpts[1].ravel()), tuple(imgpts[5].ravel()), (0,0,0), 5)
    cv.line(img, tuple(imgpts[2].ravel()), tuple(imgpts[6].ravel()), (0,0,0), 5)
    cv.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[7].ravel()), (0,0,0), 5)

    img = cv2.polylines(img, [np.int32(imgpts[4:8])],  
                      True, (0,0,0), 5)
    cv2.fillConvexPoly(img, np.int32(imgpts[4:8]), (255, 255, 255))

    return img
###############################
# used to record the time when we processed last frame 
prev_frame_time = 0
  
# used to record the time at which we processed current frame 
new_frame_time = 0

prvec = np.array([0,0,0])
ptvec = np.array([0,0,0])
##############################################################################
filt_len_rot =  5                                                            
rot_buffer = np.zeros((filt_len_rot,3),dtype=float)                                                     # anti jitter, smoothing buffer for rotation
inc_rot=0

filt_len_trs =     1                                                           
trs_buffer = np.zeros((filt_len_trs,3),dtype=float)                                                     # anti jitter,smoothing buffer for translation
inc_trs=0

##############################################################################
#webcam1 = VideoStream(src=0).start()
cap = cv.VideoCapture('monitor_emmisive.mp4')
#out = cv2.VideoWriter('project_2.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, (1920,1080))
#out = cv2.VideoWriter('filename_2.avi', cv2.VideoWriter_fourcc(*'MJPG'),15, (1920,1080)) 

while (cap.isOpened()):
    ret,frame=cap.read()
    if(ret==False):
        out.release()
    #frame =  webcam1.read()
    img=frame

    hsv =cv.cvtColor(cv.blur(img,(7,7)),cv.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, (0,100,80), (10,255,255))
    mask2 = cv2.inRange(hsv, (170,100,80), (180,255,255))

    ## Merge the mask and crop the red regions
    mask = cv2.bitwise_or(mask1, mask2 )
    #kernel = np.ones((30,30),np.uint8)
    kernel = cv2.cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))
    
    #mask = cv.dilate(mask,kernel,iterations = 1)
    #mask = cv.erode(mask,kernel,iterations = 1)
    #ret2,th2 = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)
    cv2.imshow('mask',mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    

    list_points = []

    for c in contours:
        M = cv2.moments(c)
        if(cv2.contourArea(c) > 30 and cv2.contourArea(c) < (50*50)):
            #img = cv2.drawContours(img, c, -1, (0,255,0), 3)
            
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            list_points.append(np.array([cx,cy]))

            img = cv2.circle(img, (cx,cy), radius=10, color=(0, 0, 0), thickness=-1)

    if(len(list_points)==5):
        list_points=np.array(list_points)
        ###################################################################################################################################################
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
        hullv = hull
        tanvals = np.array([])
        for k in hullv:
            xp,yp = k[0],k[1]                                                                               #custom geometry technique, varies from pattern to pattern
            tanvals = np.append(tanvals,(np.arctan2((yp-cyy),(xp-cxx))))
            print((np.arctan2((yp-cyy),(xp-cxx))))

        #print('tanvals',tanvals)

        print('hull',hull)
        #sorted_hull = hull[tanvals.argsort()]

        tanvals = np.sort(tanvals)
        #print('srt_hull',sorted_hull)

        print('tanvals',tanvals)
        print('closest',closest[0])

        #img = cv2.circle(img, (int(closest[0][0]),int(closest[0][1])), 5, (255,0,0), 10)
        comp_ang = np.arctan2((closest[0][1]-cyy),(closest[0][0]-cxx))

        index = np.where(tanvals==comp_ang )[0]

        print('index',index)
        print('before_rot',sorted_hull)
        sorted_hull=np.roll(sorted_hull,-index,axis=0)
        print('after_rot',sorted_hull)
        flag=0  
        if(len(sorted_hull)==4):
            pt1,pt2,pt3,pt4 = sorted_hull
            cv.putText(img, str("pt1"), tuple(pt1.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
            cv.putText(img, str("pt2"), tuple(pt2.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
            cv.putText(img, str("pt3"), tuple(pt3.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
            cv.putText(img, str("pt4"), tuple(pt4.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
            cv.putText(img, str("in_pt"), tuple(in_pt.ravel()), font, fontScale, color, thickness, cv.LINE_AA)
            

            dst = np.float32([pt3,pt2,pt1,pt4,in_pt])
        ########################################################################################################################################
            try:
                print(objpts.shape,dst.shape)
                _, rvecs, tvecs, inliers = cv.solvePnPRansac(objpts, dst, mtx, dist,cv.SOLVEPNP_IPPE)
                ##############################################################
                if inc_rot==filt_len_rot:
                    inc_rot=0
                rot_buffer[inc_rot]=np.squeeze(rvecs)
                inc_rot=inc_rot+1                                             # moving average filter implementation rotation
                rvecs = np.mean(rot_buffer, axis=0)
                dst, jacobian	=	cv.Rodrigues(	rvecs	)
                r=R.from_matrix(dst)
                vk = r.as_euler('zyx', degrees=True)
                ##############################################################
                #tvecs = np.squeeze(tvecs)
                if inc_trs==filt_len_trs:
                    inc_trs=0
                trs_buffer[inc_trs]=np.squeeze(tvecs)                         ## moving average filter implementation translation
                inc_trs=inc_trs+1
                tvecs = np.mean(trs_buffer, axis=0)
                #ax.scatter3D(tvecs[0], tvecs[1], tvecs[2], color = "red")
                #plt.show()
                #plt.pause(0.05)
                ###############################################################


                cv.putText(img, "R: "+str(vk), (10,400), font, fontScale, color, thickness, cv.LINE_AA)
                cv.putText(img, "T: "+str(tvecs), (10,450), font, fontScale, color, thickness, cv.LINE_AA)
                Message = str(vk)+str(tvecs)
                imgpts, jac = cv.projectPoints(cube3d, rvecs, tvecs, mtx, dist)
                img	=	cv.drawFrameAxes(	img, mtx, dist, rvecs, tvecs, length=7, thickness=10	)
                draw(img, imgpts)
        
                clientSock.sendto(bytes(Message,'utf-8'), (UDP_IP_ADDRESS, UDP_PORT_NO))
                flag=1

                
            except Exception as e: 

                print("fail",e)

        new_frame_time = time.time() 
        
        time_diff = (new_frame_time-prev_frame_time)
        #time.sleep(1)
        #fps = 1/time_diff
        fps = 30
        prev_frame_time = new_frame_time
        if(flag==1):

            vtvecs = (tvecs - ptvec)*fps
            vrvecs = (vk-prvec)*fps
            ptvec=tvecs
            prvec=vk
            cv.putText(img, "V_ang "+str(vtvecs), (20,40), font, fontScale, color, thickness, cv.LINE_AA)
            cv.putText(img, "V_tran"+str(vrvecs), (20,60), font, fontScale, color, thickness, cv.LINE_AA)
        #cv.putText(img, str((fps)), (20,20), font, fontScale, color, thickness, cv.LINE_AA)
        
    cv2.imshow('img',img)
    #out.write(img)
    cv2.waitKey(1)
#out.release()
