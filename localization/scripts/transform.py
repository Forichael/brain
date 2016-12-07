#!/usr/bin/env python  
import rospy
import tf

PI = 3.14159265358979323846264338327950

def d2r(d):
    global PI
    return d * PI / 180.

def r2d(r):
    global PI
    return r * 180 / PI

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        q = tf.transformations.quaternion_from_euler(0,0,0)
	# consider robot (base_link) orientation/position to be coincident with imu
	# need to be fixed later :/
        br.sendTransform((0.0, 0.0, 0.0),
                         q,
                         rospy.Time.now(),
                         "imu",
                         "base_link")
        rate.sleep()
