#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
from smach import *
from smach_ros import *

from move_base_msgs.msg import MoveBaseAction

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
            header=Header(frame_id='map'),
            pose=Pose(position=Point(x=x, y=y, z=0), orientation=angle))

        return dest


class Foo(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'succeeded'
        else:
            return 'aborted'


# define state Bar
class Bar(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'succeeded'


# main
def main():
    rospy.init_node('alphabot_state_machine')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'],
                      input_keys=['can_position'])

    # Open the container
    with sm:
        # Add states to the container
        StateMachine.add('FOO', Foo(),
                         transitions={'succeeded': 'BAR',
                                      'aborted': 'succeeded'})
        StateMachine.add('BAR', Bar(),
                         transitions={'succeeded': 'NAV1'})

        StateMachine.add('NAV1', Navigate(),
                         transitions={'succeeded': 'FOO',
                                      'preempted': 'aborted'},
                         remapping={'destination': 'can_position'})

        StateMachine.add('DELAY2', Delay(2),
                         transitions={'succeeded': 'FOO'})

    # Execute the machine
    data = UserData()
    data.can_position = (1, 2, 3)
    outcome = sm.execute(data)


if __name__ == '__main__':
    main()
