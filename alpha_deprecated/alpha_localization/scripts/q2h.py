#!/usr/bin/python

"""
q2h.py
Node to extract 2D Heading Information from filtered IMU quaternion orientation
"""
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu 
pub = rospy.Publisher('heading', Float32, queue_size=10)
#mg = MagneticField()

def cb(msg):
    q = msg.orientation
    h = np.arctan2(2*q.y*q.w - 2*q.x*q.z, 1-2*q.y*q.y - 2*q.z*q.z)
    pub.publish(h)

def main():
    rospy.init_node('q2h', anonymous=False)
    rospy.Subscriber('imu/data', Imu, cb)
    rospy.spin()
    
if __name__ == "__main__":
    main()
