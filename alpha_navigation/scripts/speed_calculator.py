#!/usr/bin/env python
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Twist

rospy.init_node('speed_calculator')

class SpeedCalculator(object):
    def __init__(self):
        super(SpeedCalculator, self).__init__()
        rospy.Subscriber('/odom', Odometry, self.onOdom)
        rospy.Subscriber('/cmd_vel', Twist, self.onCmd)
        self.pose = PoseWithCovariance()
        self.last_pose = PoseWithCovariance()
        self.cmd_vel = Twist()

    def onOdom(self, msg):
        """
        :type msg: Odometry
        """
        self.pose = msg.pose

    def onCmd(self, msg):
        """
        :type msg: Twist
        """
        self.cmd_vel = msg

    def run(self):
        rate = .2
        r = rospy.Rate(rate)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            dx = self.pose.pose.position.x - self.last_pose.pose.position.x
            dy = self.pose.pose.position.y - self.last_pose.pose.position.y

            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            dist = math.sqrt(dx**2 + dy**2)
            rospy.logout('Commanded speed was {0:.2f}m/s, measured was {1:.2f}m/s'.format(self.cmd_vel.linear.x, dist / dt))
            self.last_pose = self.pose

            r.sleep()

if __name__ == '__main__':
    SpeedCalculator().run()