#!/usr/bin/python

import rospy
import tf
import numpy as np

from geometry_msgs.msg import Twist, Quaternion, PoseStamped, Pose, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from pid import PID

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

from abc import ABCMeta, abstractmethod

def cap(low,x,high):
    if x < low:
        return low
    if x > high:
        return high
    return x

class PathDancerBase(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass
    @abstractmethod
    def run(self, points):
        pass

class DeadReckoningDancer(PathDancerBase):
    def __init__(self):
        super(DeadReckoningNode, self).__init__()
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_msg = Twist()
        self.state = np.zeros(3)

    def run(self, points):
        ### SETUP ALIAS NAMES ###
        l = self.cmd_msg.linear
        a = self.cmd_msg.angular
        s = self.state

        ### WAIT UNTIL CLOCK IS ALIVE ###
        r = rospy.Rate(20) # 20 hz
        start = rospy.Time.now()
        while start.to_sec() == 0:
            r.sleep()
            start = rospy.Time.now()

        ### SOME CONFIGURATION PARAMETERS ###
        dt = 5.0 # 5 sec. to reach dest

        for pt in points:
            dx,dy = pt - s[:2]
            theta = np.arctan2(dy,dx) - s[2] # angular difference
            theta = np.arctan2(np.sin(theta), np.cos(theta)) # lazy normalize
            delta = np.sqrt(dx**2+dy**2)
            w = theta/dt 
            v = delta/dt 

            ## Adjust Heading
            start = rospy.Time.now()
            a.z = w
            l.x = 0
            while (rospy.Time.now() - start).to_sec() < dt:
                self.cmd_pub.publish(self.cmd_msg)
                r.sleep()

            ## Head Towards Destination
            start = rospy.Time.now()
            a.z = 0
            l.x = v
            while (rospy.Time.now() - start).to_sec() < dt:
                self.cmd_pub.publish(self.cmd_msg)
                r.sleep()

            s[:2] = pt # update location
            s[2] = np.arctan2(dy,dx) # update heading


class OdomDancer(PathDancerBase):
    def __init__(self):
        super(OdomDancer,self).__init__()
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cmd_msg = Twist()

        self.initialized = False 
        self.initial_orientation = None

        self.position = (0,0)
        self.orientation = 0.0

        self.odom_sub = rospy.Subscriber('odometry/filtered/local', Odometry, self.odom_cb)

        self.vpid = PID(1.0,0.01,0.0)
        self.wpid = PID(1.0,0.01,0.0)

    def odom_cb(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        x = p.x
        y = p.y
        t = np.arctan2(q.z,q.w) * 2

        if not self.initialized:
            self.x0 = x
            self.y0 = y
            self.t0 = t
            self.initialized = True

        self.position = (x - self.x0, y - self.y0)
        self.orientation = t - self.t0

    def run(self, points):

        ### SETUP ALIAS NAMES ###
        l = self.cmd_msg.linear
        a = self.cmd_msg.angular

        ### SETUP CONFIGURATION ###
        r = rospy.Rate(20) # 20 hz
        dt = 1 / 20. #

        position_tolerance = 0.1
        orientation_tolerance =  np.deg2rad(5)
        while not self.initialized: 
            r.sleep()

        ### RUN CONTROL LOOP ###
        for pt in points:
            print pt

            ### ADJUST HEADING ###
            self.wpid.reset()
            while True:
                dx,dy = (pt[0] - self.position[0], pt[1] - self.position[1])
                theta = np.arctan2(dy,dx) - self.orientation # angular difference
                theta = np.arctan2(np.sin(theta), np.cos(theta)) # lazy normalize
                delta = np.sqrt(dx**2+dy**2)

                if theta < orientation_tolerance:
                    # reached goal
                    break

                w = cap(-0.5, self.wpid.compute(theta, dt), 0.5)
                a.z = w
                l.x = 0

                self.cmd_pub.publish(self.cmd_msg)
                r.sleep()

            ### REACH DESTINATION ###
            self.wpid.reset()
            self.vpid.reset()
            while True:
                dx,dy = (pt[0] - self.position[0], pt[1] - self.position[1])
                theta = np.arctan2(dy,dx) - self.orientation # angular difference
                theta = np.arctan2(np.sin(theta), np.cos(theta)) # lazy normalize
                delta = np.sqrt(dx**2+dy**2)

                if delta < position_tolerance:
                    # reached goal
                    break

                w = cap(-0.5, self.wpid.compute(theta, dt), 0.5)
                v = cap(-0.5, self.vpid.compute(delta, dt), 0.5)

                a.z = w
                l.x = v
                self.cmd_pub.publish(self.cmd_msg)
                r.sleep()

class MoveBaseDancer(PathDancerBase):
    def __init__(self):
        super(MoveBaseDancer,self).__init__()
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        pass
    def run(self, points):
        tf_listener = tf.TransformListener()
        while True:
            try:
                now = rospy.Time.now()
                tf_listener.waitForTransform('map','base_link', now, rospy.Duration(4.0))
                t,r = tf_listener.lookupTransform('map','base_link', now)
                break
            except (tf.Exception) as e:
                print e

        self.x0, self.y0 = t[0], t[1]
        self.client.wait_for_server()

        for pt in points:
            goal = self.make_goal(pt)
            self.client.send_goal_and_wait(goal)
    def make_goal(self, point):
        x,y = point 
        x += self.x0 # w.r.t. initial point
        y += self.y0
        theta = np.arctan2(y,x)
        angle = Quaternion(0, 0, np.sin(theta / 2), np.cos(theta / 2))
        dest = PoseStamped(
                header=Header(frame_id='map'),
                pose=Pose(position=Point(x=x, y=y, z=0), orientation=angle))
        goal = MoveBaseGoal(target_pose=dest)
        return goal

def main():
    rospy.init_node('alpha_dance')
    dancer = MoveBaseDancer()

    r = 2.0
    def vertex(r, i):
        theta = i * 144 * 3.1415 / 180
        return np.array([r*np.cos(theta), r*np.sin(theta)])
    points = [vertex(r,i) for i in range(6)]
    points.append(np.array([0.0,0.0])) # return to center, for convenience

    dancer.run(points)

if __name__ == "__main__":
    main()
