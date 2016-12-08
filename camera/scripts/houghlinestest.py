#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool


def imageCallback(data):
    bridge = CvBridge()
    try: 
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        img = cv_image

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 80, 150])
        upper_color = np.array([110, 130, 255])


        mask = cv2.inRange(hsv, lower_color, upper_color)
        blur = cv2.blur(mask, (7,7))
        blur = blur[0:350, 0:640]


        contours, _ = cv2.findContours(blur, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            rect = cv2.boundingRect(c)
            if rect[2] > 25 or rect[3] > 15:
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

        resized_img = img[0:200, 0:320]
        resized_mask = mask[100:200, 0:320]

        edges = cv2.Canny(resized_img,50,150,apertureSize = 3)
        minLineLength = 5000
        maxLineGap = 5
        lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)

        # if lines is not None:
        #     for x1,y1,x2,y2 in lines[0]:
        #         cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)


        cv2.imshow('mask', mask)
        cv2.imshow('edges',edges)
        cv2.imshow('original',img)
        cv2.imshow('blur', blur)

        cv2.waitKey(1)

    except CvBridgeError, e:
        print e
        print  'FAILED TO CAPTURE'


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("image_raw", Image, imageCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()