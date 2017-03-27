#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped, PolygonStamped, PointStamped, Pose, Quaternion, Point, Twist, Vector3
from std_msgs.msg import Header
from smach import *
from smach_ros import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from alpha_action.msg import GripAction, GripGoal
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskActionGoal, ExploreTaskGoal
from actionlib import SimpleActionClient
import tf


initial_point = None
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
                input_keys=['destination']
                )

        self.sub = rospy.Subscriber('/can_point', PointStamped, self.onCan) # TODO : just separate this out
        self.destination = None
        self.last_update = rospy.Time.now().to_sec()

    def onCan(self, msg):
        self.destination = tf_listener.transformPoint("base_link", msg).point
        self.last_update = rospy.Time.now().to_sec()

    def execute(self, userdata):
        print 'Beginning Navigation to Can'

        self.destination = userdata.destination
        client = SimpleActionClient('move_base', MoveBaseAction)
        
        print 'Waiting for MOVE_BASE SERVER ...'

        client.wait_for_server()
        print 'MOVE_BASE SERVER IS UP!'

        while True:
            goal = self.make_goal()
            client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5.0)) # wait 10 sec. until completion
            now = rospy.Time.now().to_sec()
            if now - self.last_update > 1.0: # more than a second has passed since sight of can
                return 'lost_can' # lost can from sight somehow
            dist = self.destination.x**2 + self.destination.y**2
            if dist < 2.0: # within 2 meters from can
                return 'succeeded'

    def make_goal(self):
        rospy.loginfo('Calculating navigation goal')

        x,y =  self.destination.x, self.destination.y
        theta = math.atan2(x,y)

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
                input_keys=['boundary'],
                output_keys=['boundary', 'can_position'])
        self.sub = rospy.Subscriber('/can_point',PointStamped,self.onCan)
        self.can_found = False
        self.can_point = None
        #SimpleActionState.__init__(
        #        self,
        #        'explore_server',
        #        ExploreTaskAction,
        #        goal_cb=self.goal_cb,
        #        )

    def onCan(self, msg):
        self.can_found = True
        self.can_point = tf_listener.transformPoint("base_link", msg).point

    def execute(self, userdata):
        print 'Beginning Exploration'
        client = SimpleActionClient('explore_server', ExploreTaskAction)
        client.wait_for_server()
        print 'SERVER IS NOW UP!'
        
        boundary = PolygonStamped()
        boundary.header.frame_id = "map"
        boundary.header.stamp = rospy.Time.now()
        r = userdata.boundary/2.0
        print 'boundary : {}'.format(r)
        x,y,_ = initial_point
        boundary.polygon.points.append(Point(x+r,y+r,0))
        boundary.polygon.points.append(Point(x-r,y+r,0))
        boundary.polygon.points.append(Point(x-r,y-r,0))
        boundary.polygon.points.append(Point(x+r,y-r,0))

        center = PointStamped() 
        center.header.frame_id = "map"
        center.header.stamp = rospy.Time.now()
        center.point.x = center.point.y = center.point.z = 0.0

        goal = ExploreTaskGoal()
        goal.explore_boundary = boundary
        goal.explore_center = center 

        client.send_goal(goal)

        while True:
            if client.wait_for_result(rospy.Duration(5.0)): # 5 sec. timeout
                # if exploration is complete...
                userdata.boundary += 1.0 # explore a larger area
                return 'succeeded' # finished!
            if self.can_found:
                client.cancel_all_goals()
                userdata.can_position = self.can_point
                print 'discovered'
                return 'discovered'
            # if we're here, then exploration is not complete yet
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
        State.__init__(self, outcomes=['succeeded'])
        self.timeout = rospy.Duration.from_sec(time)
        self.speed = speed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/can_point', PointStamped, self.onDetect)
        self.angleError = 0
        self.kp = kp

    def onDetect(self, msg):
        """
        :type msg: Point
        """

        # Positive angle error indicates robot should turn left in approximate radians
        # TODO : change this so that is performs reasonable tf transforms
        point = tf_listener.transformPoint("base_link", msg).point
        self.angleError = math.atan2(point.x,point.y)

    def execute(self, userdata):
        start_time = rospy.Time.now()
        rospy.loginfo('Beginning drive to can')
        r = rospy.Rate(10)

        # Drive the robot
        while rospy.Time.now() - start_time < self.timeout and not rospy.is_shutdown():
            turnPower = self.kp * self.angleError
            self.pub.publish(Twist(linear=Vector3(x=self.speed), angular=Vector3(z=turnPower)))
            r.sleep()

        # Stop the robot
        self.pub.publish(Twist())
        rospy.loginfo('Finished drive')
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
    global initial_point, tf_listener
    rospy.init_node('alphabot_state_machine')

    tf_listener = tf.TransformListener()

    tf_listener.waitForTransform('base_link','map', rospy.Time.now(), rospy.Time(100))
    t,r = tf_listener.lookupTransform('base_link','map', rospy.Time(0))

    initial_point = t

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'],
            input_keys=['can_position', 'boundary'])

    # Open the container
    with sm:
        # Add states to the container

        StateMachine.add('EXPLORE', Explore(),
                transitions={
                    'succeeded' : 'EXPLORE',
                    'discovered' : 'NAV1', # This should be NAV1
                    'aborted': 'EXPLORE'
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
                'NAV2', ProximityNav(speed=0.3),
                transitions={'succeeded': 'DELAY1'}
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

    outcome = sm.execute(data)


if __name__ == '__main__':
    main()
