#!/usr/bin/python

#######################TO DO:###############################
# Backlogged 1&2: --> 1a. DONE publishing center bottom pixel of can and doing ROS stuff
# 		1. need to take calibration image using alphabot camera
# 		2. pre calculate camera focal length and hardcode it into processing to save time
# 3. DONEish integrate with ROS
# 4. clean up this mess :)
# ## Figure out how to only publish frames with contours if listeners, dont do now bc info size
#


# import the necessary packages
import numpy as np
import cv2

#ROS communication
import rospy
import tf
import roslib

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped


def distance_to_camera(knownHeight, focalLength, perHeight):
    # compute and return the distance from the maker to the camera
        return (knownHeight * focalLength) / perHeight

def find_marker(image):
    #convert image into HSV, blur it
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    imageHSV = cv2.GaussianBlur(imageHSV, (9, 9), 0)
    #declare bounds for target pixel range
    lower_pink = np.array([165, 30, 10])
    higher_pink = np.array([179, 255, 255])
    #show pink pixels in range
    pinkPixels = cv2.inRange(imageHSV, lower_pink, higher_pink)

    (cnts,_) = cv2.findContours(pinkPixels.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    try: 
        #cv2.contourArea unit: pixel
        pinkContour = max(cnts, key = cv2.contourArea) 	
        if (cv2.contourArea(pinkContour) > 100):
            print "Can found"
            return cv2.boundingRect(pinkContour), pinkPixels
            #return cv2.minAreaRect(pinkContour), pinkPixels
        else:
            print "No can found (too small)"
            return 0,0
    except:
        #make a filterable exception
        #print "No can found at all"
        return 0,0

def calibrate_camera(knownDistance, knownHeight, knownImage): #maybe dont have this in
    marker,_ = find_marker(cv2.imread(knownImage))
    focalLength = marker[0][1] * KNOWN_DISTANCE / KNOWN_HEIGHT
                #I think [0][1] is can height, 2nd term, index from 0
    return focalLength

##################################START##########################################

#initialize the known (calibration) distance from the camera to the object: 24"
focalLength = 593.11 # in pixels
fov = np.deg2rad(60) # field of view
KNOWN_DISTANCE = 24.0
#initialize the known object height, which in this case, the can is 4.83" tall
KNOWN_HEIGHT = .123
KNOWN_WIDTH = 0.065 # TODO : fix this bullshit parameter
#16n5calibrate the camera for distance
# KNOWN_IMAGE = pass ########### put path to calibration image here ###############

#initialize the images aka the camera stream
# cap = cv2.VideoCapture(1)
# cv2.namedWindow('camera view')

tgt_pub = None
pub = rospy.Publisher('tgt_can_bottom', Point, queue_size=10)


def img_cb(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    result = find_marker(frame)

    if result is not None:
        minRect, pinkPixels = result
    #cv2.imshow('pink', pinkPixels)
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

        #Publish the bottom centerpoint to ROS
#        cv2.imshow('frame', frame) 
#        cv2.imshow('can', pinkPixels) 
#        cv2.waitKey(10)
##Uncomment to see, silenced for ROS:

def main():
    global tgt_pub
    #initialize ROS channels
    rospy.init_node('can_finder')
    img_sub = rospy.Subscriber('/alpha/image_raw', Image, img_cb)
    tgt_pub = rospy.Publisher('can_point', PointStamped, queue_size=10)

    #rate = rospy.Rate(20)
    rospy.spin()

    #begin infinite loop
    #while not rospy.is_shutdown():
    #    #translate from ROS format to openCV format
    #    bridge = CvBridge()
    #    cap = bridge.imgmsg_to_cv2(imgROS, desired_encoding="bgr8")

    #    #read streamed frames
    #    _,frame = cap.read()
    #    (minRect, pinkPixels) = find_marker(frame)
    #    # distTarget = distance_to_camera(KNOWN_HEIGHT, focalLength, minRect[0][1])
    #    #if target pixels were found make a contour

    #    #k = cv2.waitKey(5) & 0xFF
    #    #if k == 27:
    #    #        break

if __name__ == "__main__":
    main()

