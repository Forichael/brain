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
        q1 = tf.transformations.quaternion_from_euler(0.0, d2r(90), 0.0)
        q2 = tf.transformations.quaternion_from_euler(d2r(-90), 0.0, 0.0)
        q = tf.transformations.quaternion_multiply(q2, q1)

        br.sendTransform((0.0, 0.13, 0.0),
                         q,
                         rospy.Time.now(),
                         "camera",
                         "laser")
        rate.sleep()
