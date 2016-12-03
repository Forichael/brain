#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
recording = False
number_of_cycles = 0
breaknum = 0
c_Vel = Twist()
isInitialized = False
f = open('timelog.txt', 'w')
g = open('allinfo.txt', 'w')

def startcallback(data):
    global recording
    global number_of_cycles
    if(data.data==True):
        rospy.loginfo("Starting Video Capture")
        recording = True

def endcallback(data):
    global recording
    global number_of_cycles
    global breaknum
    if(data.data==True and recording == True):
        rospy.loginfo("Ending Video Capture")
        recording = False
        breaknum+=1
        number_of_cycles = 0


def imageCallback(data):
    global recording
    global number_of_cycles
    global f
    global c_Vel
    global isInitialized
    global g
    if(recording==True):
        bridge = CvBridge()
        rospy.loginfo("Getting Image")
        rospy.loginfo(rospy.get_time())
        if(isInitialized):
        	#f.write(str(breaknum)+','+str(number_of_cycles)+','+ str(rospy.get_time())+','+str(c_Vel.angular.z)+'\n')
	        f.write(str(breaknum)+','+str(number_of_cycles)+','+str(c_Vel.linear.x)+','+str(c_Vel.angular.z)+'\n')
	       	g.write(str(breaknum)+','+str(number_of_cycles)+','+str(c_Vel.linear.x)+','+str(c_Vel.linear.y)+','+str(c_Vel.linear.z)+','+str(c_Vel.angular.x)+','+str(c_Vel.angular.y)+','+str(c_Vel.angular.z)+'\n')
	        try: 
    	        	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        	except CvBridgeError, e:
            		print e
            		print  'FAILED TO CAPTURE'
        	else:
            		cv2.imwrite(str(breaknum)+'-'+'camera_image'+str(number_of_cycles)+'.jpeg', cv_image)
            		number_of_cycles+=1

def cmdcallback(data):
	global isInitialized
	global c_Vel
	if(isInitialized==False):
		isInitialized= True
	c_Vel = data
	print c_Vel

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global f
    f.write('session, image#, linear.x, angular.z, \n')
    g.write('session, image#, linear.x, linear.y, linear.z, angular.x, angular.y, angular.z')
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("start", Bool, startcallback)
    rospy.Subscriber("end", Bool, endcallback)
    rospy.Subscriber("cmd_vel", Twist, cmdcallback)
    rospy.Subscriber("/my_camera/image_raw", Image, imageCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()