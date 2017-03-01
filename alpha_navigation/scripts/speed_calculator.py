#!/usr/bin/env python
import math
import rospy
import numpy as np
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
        self.cmd_vel_t = rospy.Time.now()

        self.cmd_vels = []
        self.mes_vels = []

    def onOdom(self, msg):
        """
        :type msg: Odometry
        """
        self.pose = msg.pose

    def onCmd(self, msg):
        """
        :type msg: Twist
        """

        if abs(self.cmd_vel.linear.x - msg.linear.x) > 0.001: #
            # cmd_vel has changed!
            self.cmd_vel_t = rospy.Time.now()

        self.cmd_vel = msg

    def run(self):
        rate = .5
        r = rospy.Rate(rate)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            dx = self.pose.pose.position.x - self.last_pose.pose.position.x
            dy = self.pose.pose.position.y - self.last_pose.pose.position.y

            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            dist = math.sqrt(dx**2 + dy**2)

            cmd_vel = self.cmd_vel.linear.x # convert to pwm
            mes_vel = dist / dt

            if (now - self.cmd_vel_t).to_sec() > (1.0 / rate): #sufficient time has passed since last cmd_vel change
                print 'saved data! : {0:.2f} | {1:.2f}'.format(cmd_vel,mes_vel)
                # update the data set
                self.cmd_vels.append(cmd_vel)
                self.mes_vels.append(mes_vel)

            rospy.logout('Commanded speed was {0:.2f}m/s, measured was {1:.2f}m/s'.format(cmd_vel, mes_vel))
            self.last_pose = self.pose

            r.sleep()

        data = np.c_[self.cmd_vels, self.mes_vels]
        np.savetxt('calib_data.csv',data[3:],delimiter=',') # discard first reading

if __name__ == '__main__':
    SpeedCalculator().run()
