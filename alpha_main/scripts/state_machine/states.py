#!/usr/bin/env python

import rospy
from smach import *
from smach_ros import *


# define state Foo
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
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'])

    # Open the container
    with sm:
        # Add states to the container
        StateMachine.add('FOO', Foo(),
                         transitions={'succeeded': 'BAR',
                                      'aborted': 'succeeded'})
        StateMachine.add('BAR', Bar(),
                         transitions={'succeeded': 'FOO'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
