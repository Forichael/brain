#!/usr/bin/python

"""
q2h.py
Node to extract 2D Heading Information from filtered IMU quaternion orientation
"""
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField 
from geometry_msgs.msg import Vector3Stamped

imu_pub = rospy.Publisher('imu_fixed', Imu, queue_size=10)
mag_pub = rospy.Publisher('imu_fixed_mag', MagneticField, queue_size=10)

imu_msg = Imu()
mag_msg = MagneticField()

def idx(i,j,w):
    return i*w+j

def fill_header(h):
    h.stamp = rospy.Time.now()
    h.frame_id = "imu"

def imu_cb(msg):
    fill_header(imu_msg.header)
    av = msg.angular_velocity
    ac = list(msg.angular_velocity_covariance)
    la = msg.linear_acceleration
    lc = list(msg.linear_acceleration_covariance)

    xi = idx(0,0,3)
    yi = idx(1,1,3)

    av.x,av.y,av.z = av.y,av.x,-av.z# swap x,y and negate z 
    ac[xi],ac[yi] = ac[yi],ac[xi]

    la.x,la.y,la.z = la.y,la.x,-la.z # swap x,y and negate z
    lc[xi],lc[yi] = lc[yi],lc[xi]

    imu_msg.angular_velocity = av
    imu_msg.angular_velocity_covariance = ac
    imu_msg.linear_acceleration = la
    imu_msg.linear_acceleration_covariance = lc
    imu_pub.publish(imu_msg)

def mag_cb(msg):
    fill_header(mag_msg.header)
    v = msg.vector
    v.x,v.y,v.z = v.y,v.x,v.z #swap x,y, and negate z
    mag_msg.magnetic_field = v
    mag_pub.publish(mag_msg)

def main():
    rospy.init_node('q2h', anonymous=False)
    rospy.Subscriber('imu/data_raw', Imu, imu_cb)
    rospy.Subscriber('imu/mag', Vector3Stamped, mag_cb)
    rospy.spin()
    
if __name__ == "__main__":
    main()
