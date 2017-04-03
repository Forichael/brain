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

#ROS communication
import rospy
import tf
import roslib

from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, PointStamped

dis_pub = None
del_pub = None

def to_msg(d): # convert detection to PointStamped message
    res = PointStamped()
    res.header.frame_id = 'camera'
    res.header.stamp = d.pose.header.stamp
    p = d.pose.pose.position
    res.point.x = p.z
    res.point.y = -p.x
    res.point.z = -p.y
    return res

def tag_cb(msg):
    for d in msg.detections:
        p_msg = to_msg(d)
        if d.id == 0:
            dis_pub.publish(p_msg) # discovery point
        elif d.id == 1:
            del_pub.publish(p_msg) # delivery point

def main():
    global dis_pub, del_pub
    #initialize ROS channels
    rospy.init_node('april_tag_finder')
    img_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_cb)
    dis_pub = rospy.Publisher('/dis_pt', PointStamped, queue_size=10) # pretend to be can
    del_pub = rospy.Publisher('/del_pt', PointStamped, queue_size=10) # pretend to be can
    rospy.spin()

if __name__ == "__main__":
    main()
