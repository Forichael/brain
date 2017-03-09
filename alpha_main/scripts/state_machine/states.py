#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, Twist, Vector3
from std_msgs.msg import Header
from smach import *
from smach_ros import *

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from alpha_action.msg import GripAction, GripGoal


class Delay(State):
    def __init__(self, delay_time=1):
        State.__init__(self, outcomes=['succeeded'])
        self.delay_time = delay_time

    def execute(self, userdata):
        rospy.loginfo('Beginning {}s delay'.format(self.delay_time))

        rospy.sleep(self.delay_time)

        return 'succeeded'


class Navigate(SimpleActionState):
    """
    Navigate is a state that calls the move_base action
    with an argument "destination" passed as a tuple (x, y, theta)
    """

    def __init__(self):
        SimpleActionState.__init__(self,
                                   'move_base',
                                   MoveBaseAction,
                                   goal_cb=self.goal_cb,
                                   input_keys=['destination'])

    def goal_cb(self, userdata, goal):
        rospy.loginfo('Calculating navigation goal')

        theta = userdata.destination[2]
        x, y = userdata.destination[:2]
        angle = Quaternion(0, 0, math.sin(theta / 2), math.cos(theta / 2))

        dest = PoseStamped(
            header=Header(frame_id='base_link'),
            pose=Pose(position=Point(x=x, y=y, z=0), orientation=angle))

        return MoveBaseGoal(target_pose=dest)


def Grip():
    gripper_goal = GripGoal()
    gripper_goal.do_grip = True
    return SimpleActionState('alpha_grip', GripAction, goal=gripper_goal)


class ProximityNav(State):
    def __init__(self, time=6, speed=0.2):
        State.__init__(self, outcomes=['succeeded'])
        self.timeout = rospy.Duration.from_sec(time)
        self.speed = speed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):
        startTime = rospy.Time.now()
        rospy.loginfo('Beginning drive to can')
        r = rospy.Rate(10)

        # Drive the robot
        while rospy.Time.now() - startTime < self.timeout and not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.speed)))
            r.sleep()

        # Stop the robot
        self.pub.publish(Twist())
        rospy.loginfo('Finished drive')
        return 'succeeded'


class Foo(State):
    """
    Foo runs several times, ending with returning "aborted"
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
    rospy.init_node('alphabot_state_machine')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'],
                      input_keys=['can_position'])

    # Open the container
    with sm:
        # Add states to the container
        # StateMachine.add('FOO', Foo(),
        #                  transitions={'succeeded': 'NAV1',
        #                               'aborted': 'succeeded'})

        StateMachine.add('NAV1', Navigate(),
                         transitions={'succeeded': 'NAV2',
                                      'preempted': 'aborted'},
                         remapping={'destination': 'can_position'})

        StateMachine.add('NAV2', ProximityNav(),
                         transitions={'succeeded': 'GRIP'})

        StateMachine.add('GRIP', Grip(),
                         transitions={'succeeded': 'DELAY2',
                                      'preempted': 'aborted'})

        StateMachine.add('DELAY2', Delay(2),
                         transitions={'succeeded': 'succeeded'})

        sm.set_initial_state(['NAV1'])

    # Execute the machine
    data = UserData()
    data.can_position = (1, 0, 0)
    outcome = sm.execute(data)


if __name__ == '__main__':
    main()
