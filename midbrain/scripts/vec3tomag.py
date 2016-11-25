#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import MagneticField

pub = rospy.Publisher('imu/magnetic_field', MagneticField, queue_size=10)
mg = MagneticField()

def cb(msg):
    global mg, pub
    mg.header.stamp = rospy.Time.now()
    mg.header.frame_id = "imu"
    mg.magnetic_field = msg.vector
    pub.publish(mg)

def main():
    rospy.init_node('vec3tomag', anonymous=False)
    rospy.Subscriber('imu/mag', Vector3Stamped, cb)
    rospy.spin()
    
if __name__ == "__main__":
    main()
