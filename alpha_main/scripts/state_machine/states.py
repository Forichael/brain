#!/usr/bin/env python
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PolygonStamped, PointStamped, Pose, Quaternion, Point, Twist, Vector3
from std_msgs.msg import Header
from smach import *
from smach_ros import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from alpha_action.msg import GripAction, GripGoal
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskActionGoal, ExploreTaskGoal
from actionlib import SimpleActionClient
import tf


## Try to filter the can's position by just removing outliers and taking the mean ...
## Caution here is the assumption that the map stays relatively correct & stable over time
def MahalanobisDist(x, y):
    covariance_xy = np.cov(x,y, rowvar=0)
    inv_covariance_xy = np.linalg.inv(covariance_xy)
    xy_mean = np.mean(x),np.mean(y)
    x_diff = np.array([x_i - xy_mean[0] for x_i in x])
    y_diff = np.array([y_i - xy_mean[1] for y_i in y])
    diff_xy = np.transpose([x_diff, y_diff])
    
    md = []
    for i in range(len(diff_xy)):
        md.append(np.sqrt(np.dot(np.dot(np.transpose(diff_xy[i]),inv_covariance_xy),diff_xy[i])))
    return md

def MD_removeOutliers(x, y):
    MD = MahalanobisDist(x, y)
    threshold = np.mean(MD) * 1.5 # adjust 1.5 accordingly 
    nx, ny, outliers = [], [], []
    for i in range(len(MD)):
        if MD[i] <= threshold:
            nx.append(x[i])
            ny.append(y[i])
        else:
            outliers.append(i) # position of removed pair
    return (np.array(nx), np.array(ny))

def filtered_can_point(xs,ys):
    xs,ys = MD_removeOutliers(xs,ys)
    x,y = (np.mean((xs,ys), axis=1))
    return Point(x,y,0)

filtered_can_point_pub = None

tf_listener = None

class Delay(State):
    def __init__(self, delay_time=1):
        State.__init__(self, outcomes=['succeeded'])
        self.delay_time = delay_time

    def execute(self, userdata):
        rospy.loginfo('Beginning {}s delay'.format(self.delay_time))
        rospy.sleep(self.delay_time)
        return 'succeeded'

class Navigate(State):
    """
    Navigate is a state that calls the move_base action
    with an argument "destination" passed as a point (x,y,z) as in position.
    """

    def __init__(self):
        State.__init__(self,
                outcomes=['succeeded','lost_can','aborted'],
                input_keys=['destination'],
                output_keys=['initial_point']
                )
        self.destination = Point()

        self.dests_x = []
        self.dests_y = []

        self.dest_in_map = Point()
        self.last_update = rospy.Time.now().to_sec()

    def onCan(self, msg):
        global filtered_can_point_pub
        n_attempts = 10
        for _ in range(10):
            try:
                now = rospy.Time.now()
                tf_listener.waitForTransform('map','base_link', now, rospy.Duration(1.0))
                dest = tf_listener.transformPoint("base_link", msg)
                self.destination = dest.point
                p = tf_listener.transformPoint("map", msg).point

                self.dests_x.append(p.x)
                self.dests_y.append(p.y)

                if len(self.dests_x) > 10:
                    self.dest_in_map = filtered_can_point(self.dests_x, self.dests_y)
                    f_p = PointStamped() 
                    f_p.header.frame_id = "map"
                    f_p.header.stamp = now 
                    f_p.point = self.dest_in_map
                    filtered_can_point_pub.publish(f_p)
                    self.destination = tf_listener.transformPoint("base_link", f_p).point
                else:
                    self.dest_in_map = p

                self.last_update = now.to_sec()
                return
            except (tf.Exception, tf.ExtrapolationException) as e:
                pass

    def execute(self, userdata):

        self.sub = rospy.Subscriber('/can_point', PointStamped, self.onCan) # TODO : just separate this out
        rospy.loginfo('Beginning Navigation to Can')

        self.destination = userdata.destination
        client = SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo('Waiting for MOVE_BASE SERVER ...')

        client.wait_for_server()
        rospy.loginfo('MOVE_BASE SERVER IS UP!')

        while True:
            goal = self.make_goal()
            client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5.0)) # wait 10 sec. until completion
            now = rospy.Time.now().to_sec()

            p = self.dest_in_map
            if p.x != 0 and p.y != 0:
                print 'setting initial point : xyz {} {} {}'.format(p.x,p.y,p.z)
                # next time, search around can area
                userdata.initial_point = [p.x, p.y, p.z]

            if now - self.last_update > 5.0: # more than 5 seconds have passed since sight of can
                self.sub.unregister()
                client.cancel_all_goals()

                return 'lost_can' # lost can from sight somehow
            dist = self.destination.x**2 + self.destination.y**2
            if dist < 2.0: # within 4 meters from can
                self.sub.unregister()
                client.cancel_all_goals() #start manual drive!
                return 'succeeded'

    def make_goal(self):
        rospy.loginfo('Calculating navigation goal')

        x,y =  self.destination.x, self.destination.y
        theta = math.atan2(y,x)

        r = 0.5 # 50cm back from the can position
        x -= r * math.cos(theta)
        y -= r * math.sin(theta)

        angle = Quaternion(0, 0, math.sin(theta / 2), math.cos(theta / 2))

        dest = PoseStamped(
                header=Header(frame_id='base_link'),
                pose=Pose(position=Point(x=x, y=y, z=0), orientation=angle))

        goal = MoveBaseGoal(target_pose=dest)
        return goal


class Explore(State):
    def __init__(self):
        State.__init__(self,
                outcomes=['succeeded', 'discovered', 'aborted'],
                input_keys=['boundary', 'initial_point'],
                output_keys=['boundary', 'can_position'])
        self.can_point = None
        self.can_time = None 
        #SimpleActionState.__init__(
        #        self,
        #        'explore_server',
        #        ExploreTaskAction,
        #        goal_cb=self.goal_cb,
        #        )

    def onCan(self, msg):
        self.can_point = tf_listener.transformPoint("base_link", msg).point
        self.can_time = msg.header.stamp

    def execute(self, userdata):
        self.sub = rospy.Subscriber('/can_point',PointStamped,self.onCan)

        rospy.loginfo('Beginning Exploration')
        client = SimpleActionClient('explore_server', ExploreTaskAction)
        rospy.loginfo('WAITING FOR EXPLORE SERVER...')
        client.wait_for_server()
        rospy.loginfo('EXPLORE SERVER IS NOW UP!')
        
        boundary = PolygonStamped()
        boundary.header.frame_id = "map"
        boundary.header.stamp = rospy.Time.now()
        r = userdata.boundary/2.0
        rospy.loginfo('boundary : {}'.format(r))
        x,y,_ = userdata.initial_point
        boundary.polygon.points.append(Point(x+r,y+r,0))
        boundary.polygon.points.append(Point(x-r,y+r,0))
        boundary.polygon.points.append(Point(x-r,y-r,0))
        boundary.polygon.points.append(Point(x+r,y-r,0))

        center = PointStamped() 
        center.header.frame_id = "map"
        center.header.stamp = rospy.Time.now()
        center.point.x = x
        center.point.y = y
        center.point.z = 0.0

        goal = ExploreTaskGoal()
        goal.explore_boundary = boundary
        goal.explore_center = center 

        client.send_goal(goal)

        while True:
            if client.wait_for_result(rospy.Duration(0.3)): # 0.3 sec. timeout to check for cans
                res = client.get_state()
                rospy.loginfo('EXPLORE SERVER STATE:{}'.format(res))
                if res == 3: ## SUCCEEDED
                    # if exploration is complete...
                    userdata.boundary += 1.0 # explore a larger area
                    self.sub.unregister()
                    return 'succeeded' # finished! yay!
                elif res == 4: ## "ABORTED" ?? keep trying...
                    self.sub.unregister()
                    return 'succeeded'
                else:
                    # when explore server gives up, can't explore
                    return 'aborted'
            #exploration is not complete yet...
            if self.can_time != None: # check initialized
                can_found = (rospy.Time.now() - self.can_time).to_sec() < 0.3
                if can_found:
                    # if can was found ...
                    client.cancel_all_goals()
                    userdata.can_position = self.can_point
                    self.sub.unregister()
                    return 'discovered'
            # if we're here, then exploration is not complete yet
        self.sub.unregister()
        return 'aborted'

    #def goal_cb(self, userdata, goal):
    #    boundary = PolygonStamped()
    #    boundary.header.frame_id = "base_link"
    #    boundary.header.stamp = rospy.Time.now()

    #    boundary.polygon.points.append(Point(5,5,0))
    #    boundary.polygon.points.append(Point(-5,5,0))
    #    boundary.polygon.points.append(Point(-5,-5,0))
    #    boundary.polygon.points.append(Point(5,-5,0))

    #    center = PointStamped() 
    #    center.header.frame_id = "base_link"
    #    center.header.stamp = rospy.Time.now()
    #    center.point.x = center.point.y = center.point.z = 0.0

    #    #goal = ExploreTaskActionGoal()
    #    #goal.header.frame_id = "base_link"
    #    #goal.header.stamp = rospy.Time.now()

    #    goal = ExploreTaskGoal()
    #    goal.explore_boundary = boundary
    #    goal.explore_center = center 

    #    return goal



def Grip(close=True):
    gripper_goal = GripGoal()
    gripper_goal.do_grip = close
    return SimpleActionState('alpha_grip', GripAction, goal=gripper_goal)


class ProximityNav(State):
    def __init__(self, time=6, speed=0.2, kp=1.0/200):
        State.__init__(self, outcomes=['succeeded','lost_can'])
        self.timeout = rospy.Duration.from_sec(time)
        self.max_speed = speed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angleError = 0
        self.kp = kp

        self.last_detected = rospy.Time.now()

    def onDetect(self, msg):
        """
        :type msg: Point
        """
        # Positive angle error indicates robot should turn left in approximate radians
        # TODO : change this so that is performs reasonable tf transforms
        point = tf_listener.transformPoint("base_link", msg).point
        dist = math.sqrt(point.x**2+point.y**2)
        self.angleError = math.atan2(point.y,point.x)

        self.speed = 0.05 * dist #adjust w.r.t. distance

        if self.speed > self.max_speed:
            self.speed = self.max_speed

        rospy.loginfo('Angle Error : {}; Distance : {}'.format(self.angleError, dist))
        self.last_detected = msg.header.stamp

    def execute(self, userdata):
        self.sub = rospy.Subscriber('/can_point', PointStamped, self.onDetect)
        self.speed = self.max_speed

        start_time = rospy.Time.now()
        rospy.loginfo('Beginning drive to can')
        r = rospy.Rate(10)

        # Drive the robot
        while rospy.Time.now() - start_time < self.timeout and not rospy.is_shutdown():
            turnPower = self.kp * self.angleError
            self.pub.publish(Twist(linear=Vector3(x=self.speed), angular=Vector3(z=turnPower)))
            r.sleep()
            if (rospy.Time.now() - self.last_detected).to_sec() > 0.5:
                self.sub.unregister()
                return 'lost_can'

        # Stop the robot
        self.pub.publish(Twist())
        rospy.loginfo('Finished drive')
        self.sub.unregister()
        return 'succeeded'


class Backup(State):
    def __init__(self, time=6, speed=-0.2):
        State.__init__(self, outcomes=['succeeded'])
        self.timeout = rospy.Duration.from_sec(time)
        self.speed = speed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        start_time = rospy.Time.now()
        rospy.loginfo('Beginning backup')
        r = rospy.Rate(10)

        # Drive the robot
        while rospy.Time.now() - start_time < self.timeout and not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.speed)))
            r.sleep()

        # Stop the robot
        self.pub.publish(Twist())
        rospy.loginfo('Finished drive')
        return 'succeeded'


class Loop(State):
    """
    Loop runs several times, ending with returning "aborted"
    """

    def __init__(self, loops=3):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.counter = loops

    def execute(self, userdata):
        if self.counter > 0:
            self.counter -= 1
            return 'succeeded'
        else:
            return 'aborted'


# define state Bar
# class Bar(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['succeeded'])
#
#     def execute(self, userdata):
#         rospy.loginfo('Executing state BAR')
#         return 'succeeded'


# main
def main():
    global tf_listener, filtered_can_point_pub
    rospy.init_node('alphabot_state_machine')
    rospy.loginfo('waiting for map->base_link tf transform ...')

    filtered_can_point_pub = rospy.Publisher('/filtered_can_point', PointStamped, queue_size=10)

    tf_listener = tf.TransformListener()
    t,r = None,None

    while True:
        try:
            now = rospy.Time.now()
            tf_listener.waitForTransform('map','base_link', now, rospy.Duration(4.0))
            t,r = tf_listener.lookupTransform('map','base_link', now)
            break
        except (tf.Exception) as e:
            print e

    rospy.loginfo('tf done!')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'],
            input_keys=['can_position', 'initial_point', 'boundary'])

    # Open the container
    with sm:
        # Add states to the container

        StateMachine.add('EXPLORE', Explore(),
                transitions={
                    'succeeded' : 'EXPLORE',
                    'discovered' : 'NAV1', # This should be NAV1
                    'aborted': 'aborted'
                    },
                remapping={'boundary':'boundary'} # keep on exploring with larger areas until can is found!
                )

        StateMachine.add('RELEASE1', Grip(False),
                transitions={'succeeded': 'LOOP',
                    'preempted': 'aborted'}
                )

        StateMachine.add('LOOP', Loop(),
                transitions={
                    'succeeded': 'NAV2',
                    'aborted': 'succeeded'
                    }
                )

        StateMachine.add('NAV1', Navigate(),
                         transitions={'succeeded': 'NAV2',
                                      'lost_can': 'EXPLORE',
                                      'aborted': 'EXPLORE'
                                      },
                         remapping={'destination': 'can_position'})

        StateMachine.add(
                'NAV2', ProximityNav(speed=0.2),
                transitions={
                    'succeeded': 'DELAY1',
                    'lost_can': 'EXPLORE'
                    }
                )

        StateMachine.add(
                'DELAY1', Delay(2),
                transitions={'succeeded': 'GRIP'}
                )

        StateMachine.add('GRIP', Grip(True),
                transitions={'succeeded': 'DELAY2',
                    'preempted': 'aborted'}
                )

        StateMachine.add('DELAY2', Delay(2),
                transitions={'succeeded': 'BACKUP1'})

        StateMachine.add('BACKUP1', Backup(time=3),
                transitions={'succeeded': 'RELEASE2'})

        StateMachine.add('RELEASE2', Grip(False),
                transitions={
                    'succeeded': 'BACKUP2',
                    'preempted': 'aborted'
                    }
                )

        StateMachine.add('BACKUP2', Backup(time=5),
                transitions={'succeeded': 'LOOP'})


        #sm.set_initial_state(['RELEASE1'])
        sm.set_initial_state(['EXPLORE'])

    # Execute the machine
    data = UserData()

    data.can_position = Point(1,0,0) 
    data.boundary = 3.0 # start out with 3m x 3m boundary exploration for can
    data.initial_point = t


    sis = IntrospectionServer('smach', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute(data)

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
