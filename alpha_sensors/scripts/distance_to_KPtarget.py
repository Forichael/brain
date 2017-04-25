cd#!/usr/bin/python

# import the necessary packages
import numpy as np
import cv2
import os

#ROS communication
import rospy
import tf
import roslib

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
#include import msg type for whatever KP and images are getting published as

def distance_to_camera(knownHeight, focalLength, perHeight):
    # compute and return the distance from the maker to the camera
        return (knownHeight * focalLength) / perHeight

def find_marker(image):
    #convert image into HSV, blur it
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    imageHSV = cv2.GaussianBlur(imageHSV, (9, 9), 0)
    #declare bounds for target pixel range
    lower_HSV = np.array([30, 50, 0]) #([70, 100, 10])  ###Outdoor/indoor 7up can
    higher_HSV = np.array([80, 255, 255]) #([100, 255, 255])
    #show color pixels in range
    colorPixels = cv2.inRange(imageHSV, lower_HSV, higher_HSV)

    (cnts,_) = cv2.findContours(colorPixels.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    try: 
        #cv2.contourArea unit: pixel
        colorContour = max(cnts, key = cv2.contourArea)     
        if (cv2.contourArea(colorContour) > 100):
            print "Can color found"
            return cv2.boundingRect(colorContour), colorPixels
            #return cv2.minAreaRect(colorContour), colorPixels
        else:
            print "No can color found (too small)"
            return 0,0
    except:
        #make a filterable exception
        #print "No can found at all"
        return 0,0

def find_KPmatches(img1, des1, kp1, img2, des2, kp2):
        #Make sure that the descriptors are correct format for knnMatch
        #checks the type of array, and recasts array as type float32 if not already
        try:
            if(des1.dtype != np.float32):
                des1 = des1.astype(np.float32)
            if(des2.dtype != np.float32):
                des2 = des2.astype(np.float32)

            matches = flann.knnMatch(des1, des2, k=2)

            #Filter for good matches
            good = []
            # dist = []
            # ratio test as per Lowe's paper
            for i,(m,n) in enumerate(matches):
                if m.distance < 0.7*n.distance:
                    good.append(m)
                    # dist.append(n)
            return good, len(good)
                
        except:
            return [], 0

def calibrate_camera(knownDistance, knownHeight, knownImage): #maybe dont have this in
    marker,_ = find_marker(cv2.imread(knownImage))
    focalLength = marker[0][1] * KNOWN_DISTANCE / KNOWN_HEIGHT
                #I think [0][1] is can height, 2nd term, index from 0
    return focalLength

##################################START##########################################

#Initialize distance info
focalLength = 593.11 # in pixels
fov = np.deg2rad(60) # field of view
KNOWN_DISTANCE = 24.0
KNOWN_HEIGHT = .123
KNOWN_WIDTH = 0.065 # TODO : fix this bullshit parameter
#16n5calibrate the camera for distance

#Initialize orb type descriptors
orb = cv2.ORB()
min_matches = 1 #minimum number of KP matches
num_train = 11 #number of training images
#FLANN parameters
FLANN_INDEX_KDTREE = 0;
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50) #or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)
minReqMatches = 4 #Minimum amount of KP matches to ID can
#Initialize training image info
# operating software, open control line, enter string, read 
# and return results, remove leading or trailing whitespace
sensor_dir = os.popen('rospack find alpha_sensors').read().strip()
#Initialize training images
img_path = sensor_dir+'/data/7upCanCrop/'
img_train = []
kp_train = []
des_train = []

#fill the training lists using the training images
for i in range(1,num_train+1):
    # print(img_path+'7up'+ str(i) +'.jpg')
    img = cv2.imread(img_path+'7up'+ str(i) +'.jpg')
    img = cv2.resize(img,dsize=(0,0), fx=0.35, fy=0.35) #make img smaller

    if img is not None:
        kp, des = orb.detectAndCompute(img, None)

        img_train.append(img)
        kp_train.append(kp)
        des_train.append(des)

#initialize the images aka the camera stream
# cap = cv2.VideoCapture(1)
# cv2.namedWindow('camera view')

tgt_pub = None
pub = rospy.Publisher('tgt_can_bottom', Point, queue_size=10)
# kp_pub =None
# pub = rospy.Publisher('kp_can_info', Type, queue_size=10)

def img_cb(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    result = find_marker(frame)

    if result is not None:
        minRect, colorPixels = result
    #cv2.imshow('color', colorPixels)
    #cv2.waitKey(10)

    x,y,z = 0,0,0

    if (minRect != 0):
        #obtain the info from Rect to draw a box and draw it on the frame
        #box = np.int0(cv2.cv.BoxPoints(minRect))
        #cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        print minRect
        x,y,w,h  = minRect
        cv2.circle(frame, (int(x), int(y+h/2)), 10, (255, 0, 0), 3)

        ptCenBot = [x, y+h/2]

        #check for keypoints inside the box (region of interest)
        ROI = frame[np.floor(x-w/2):np.floor(x+w/2),np.floor(y-h/2):np.floor(y+h/2)]
        kp_ROI, des_ROI = orb.detectAndCompute(ROI,None)
        bestMatches = []
        bestNumMatches = 0
        bestTrainImg = 0
        for t in range(1, num_train+1):
            matches, numMatches = find_KPmatches(ROI, des_ROI, kp_ROI, \
                                        img_train[t], des_train[t], kp_train[t])
            #only hold onto the best match, if any
            if numMatches > bestNumMatches:
                bestNumMatches = numMatches #number of KP matches made
                bestMatches = matches #list of matched points
                bestTrainImg = t #index of the best matched training image

        # Could publish number of matches associated with the best contour,
        #   if less than threshhold notes location and keeps looking
        # For human viewing need matches, ROI, img_train[bestTrainingImg]
        #   and the respective keypoints. If not ROI then minRect.

        if bestNumMatches > minReqMatches:


            #Publish the bottom centerpoint to ROS
            pub.publish(Point(x=ptCenBot[0], y=ptCenBot[1]))

            # 640x480
            dx = x - 320
            dy = y - 240

            theta = np.arctan(dx/focalLength) # yaw-ish
            phi = np.arctan(dy/focalLength) # pitch

            distance = distance_to_camera(KNOWN_WIDTH, focalLength, w) # distance
            print 'width', w
            print 'dist', distance

            tgt_msg = PointStamped()

            tgt_msg.header.frame_id = 'camera'
            tgt_msg.header.stamp = rospy.Time.now()

            tgt_msg.point.x = distance
            tgt_msg.point.y = -distance * np.tan(theta) # because y is pointed left in ros, flipped
            tgt_msg.point.z = -distance * np.tan(phi) # because camera coordinates, y is flipped

            tgt_pub.publish(tgt_msg)
            #br = tf.TransformBroadcaster()
            #br.sendTransform((distance,-distance*np.tan(theta),0), #don't really care abt "up" direction;  x is forwards direction. sign of y is flipped because in ros y is pointed left
            #        tf.transformations.quaternion_from_euler(0,0,0), #don't care
            #        rospy.Time.now(),
            #        'can',
            #        'camera'
            #        )

        else:
            ptCen = [0,0]
            ptCenBot = [0,0]
        
    else:
        ptCen = [0,0]
        ptCenBot = [0,0]

        #Publish the bottom centerpoint to ROS
#        cv2.imshow('frame', frame) 
#        cv2.imshow('can', colorPixels) 
#        cv2.waitKey(10)
##Uncomment to see, silenced for ROS:

def main():
    global tgt_pub
    #initialize ROS channels
    rospy.init_node('can_finder')
    img_sub = rospy.Subscriber('/alpha/image_raw', Image, img_cb)
    tgt_pub = rospy.Publisher('can_point', PointStamped, queue_size=10)
    # kp_pub = rospy.Publisher('kp_can_info', Type, queue_size=10)

    #rate = rospy.Rate(20)
    rospy.spin()

    #begin infinite loop
    #while not rospy.is_shutdown():
    #    #translate from ROS format to openCV format
    #    bridge = CvBridge()
    #    cap = bridge.imgmsg_to_cv2(imgROS, desired_encoding="bgr8")

    #    #read streamed frames
    #    _,frame = cap.read()
    #    (minRect, colorPixels) = find_marker(frame)
    #    # distTarget = distance_to_camera(KNOWN_HEIGHT, focalLength, minRect[0][1])
    #    #if target pixels were found make a contour

    #    #k = cv2.waitKey(5) & 0xFF
    #    #if k == 27:
    #    #        break

if __name__ == "__main__":
    main()

