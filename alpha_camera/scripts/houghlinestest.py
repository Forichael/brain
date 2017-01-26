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
from math import exp
from std_msgs.msg import Float32MultiArray

desired_angle = [0 for _ in range(11)]

def imageCallback(data):
    global desired_angle
    bridge = CvBridge()
    try: 
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        img = cv_image

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 80, 150])
        upper_color = np.array([110, 130, 255])


        mask = cv2.inRange(hsv, lower_color, upper_color)
        blur = cv2.blur(mask, (7,7))
        
        upper_offset = 100
        blur = blur[upper_offset:350, 0:640]
        
        contours, _ = cv2.findContours(blur, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        mass_list = []
        for c in contours:
            rect = cv2.boundingRect(c)
            if rect[2] > 25 or rect[3] > 15:
                x,y,w,h = cv2.boundingRect(c)
                y += upper_offset
                area = w*h
                center = (x+w/2, y+h/2)
                mass_list.append((area, center,h))
                
                cv2.circle(img, center, 5, (255, 0,0),7)
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        #print(mass_list)
        cm = [0,0]
        mass_total = 0

        for i in range(len(mass_list)):
            area = mass_list[i][0]*((350-upper_offset-center[1]+mass_list[i][2])**2)
         #   print(area)
            mass_total += area
            center = mass_list[i][1]
            center_x = center[0]
            center_y = center[1]
            cm[0] += center_x*area
            cm[1] += center_y*area
        if mass_total > 0:
            cm[0] = cm[0]/mass_total
            cm[1] = cm[1]/mass_total
            cv2.circle(img, (cm[0], cm[1]), 25, (0, 0, 255), 7)
            desired_x = cm[0]/640.0*10.0
            #print desired_x
            desired_angle =[]
            total_confidence = 0            

            for a in range(0,11):
                desired_angle.append(1*exp(-(a-desired_x)**2/2.0)*(350-upper_offset-cm[1])**2/((350.0-upper_offset)**2))
                
            #print (desired_angle)
        else:
            desired_angle=[]
            for a in range(0,11):
                desired_angle.append(0)
        resized_img = img[0:200, 0:320]
        resized_mask = mask[100:200, 0:320]

        edges = cv2.Canny(resized_img,50,150,apertureSize = 3)
        minLineLength = 5000
        maxLineGap = 5
        lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
        # if lines is not None:
        #     for x1,y1,x2,y2 in lines[0]:
        #         cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
 

        #cv2.imshow('mask', mask)
        #cv2.imshow('edges',edges)
        #cv2.imshow('original',img)
        #cv2.imshow('blur', blur)

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
    pub = rospy.Publisher('cones/cmd_vel', Float32MultiArray,queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Float32MultiArray()
        msg.data.append([i/22. for i in range(11)])
        msg.data.append(desired_angle)
        msg.data = [y for x in msg.data for y in x]
        pub.publish(msg)
        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
