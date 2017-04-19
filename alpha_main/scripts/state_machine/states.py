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
from topic_tools.srv import MuxSelect
import tf


## Try to filter the can's position by just removing outliers and taking the mean ...
## Caution here is the assumption that the map stays relatively correct & stable over time
def MahalanobisDist(x, y):
    covariance_xy = np.cov(x, y, rowvar=0)
    inv_covariance_xy = np.linalg.inv(covariance_xy)
    xy_mean = np.mean(x), np.mean(y)
    x_diff = np.array([x_i - xy_mean[0] for x_i in x])
    y_diff = np.array([y_i - xy_mean[1] for y_i in y])
    diff_xy = np.transpose([x_diff, y_diff])

    md = []
    for i in range(len(diff_xy)):
        md.append(np.sqrt(np.dot(np.dot(np.transpose(diff_xy[i]), inv_covariance_xy), diff_xy[i])))
    return md


def MD_removeOutliers(x, y):
    MD = MahalanobisDist(x, y)
    threshold = np.mean(MD) * 1.5  # adjust 1.5 accordingly
    nx, ny, outliers = [], [], []
    for i in range(len(MD)):
        if MD[i] <= threshold:
            nx.append(x[i])
            ny.append(y[i])
        else:
            outliers.append(i)  # position of removed pair
    return (np.array(nx), np.array(ny))


def filtered_point(xs, ys):
    if len(xs) > 10:
        xs, ys = MD_removeOutliers(xs, ys)
        x, y = (np.mean((xs, ys), axis=1))
        return Point(x, y, 0)
    else:
        return Point(xs[-1], ys[-1], 0)


tf_listener = None
ms = None


class PointData(object):
    """
    A PointData stores a filtered point and the history of readings leading up to that
    name : string
    points : {'x' : list(float), 'y' : list(float)}
    point : geometry_msgs/Point (in 'map' frame)
    time : std_msgs/Time
    """

    # TODO: store the orientation of the tag so we can go to it
    def __init__(self, name=''):
        self.name = name
        self.points = {'x': [], 'y': []}
        self.point = None
        self.time = None

    def update(self, pt, msg):
        self.points['x'].append(pt.x)
        self.points['y'].append(pt.y)
        self.point = filtered_point(self.points['x'], self.points['y'])
        self.time = msg.header.stamp


class MissionSubscriber(object):
    """
    Class that subscribes to each of the critical locations (/dis_pt, /del_pt, /can_point)
    and harnesses their filtered data.
    """

    def __init__(self):
        self.dis_sub = rospy.Subscriber('/dis_pt', PointStamped, self.onDis)
        self.del_sub = rospy.Subscriber('/del_pt', PointStamped, self.onDel)
        self.can_sub = rospy.Subscriber('/can_point', PointStamped, self.onCan)

        self.dis_data = PointData('dis')
        self.del_data = PointData('del')
        self.can_data = PointData('can')

    def convert(self, msg):
        try:
            now = rospy.Time.now()
            tf_listener.waitForTransform('map', msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            p = tf_listener.transformPoint("map", msg).point
            return p, True
        except (tf.Exception, tf.ExtrapolationException, tf.LookupException) as e:
            # print e
            return None, False

    def onPt(self, msg, data):
        pt, succ = self.convert(msg)
        if succ:
            data.update(pt, msg)

    def onDis(self, msg):
        self.onPt(msg, self.dis_data)

    def onDel(self, msg):
        self.onPt(msg, self.del_data)

    def onCan(self, msg):
        self.onPt(msg, self.can_data)

    def dis_pt(self):
        return self.dis_data.point

    def del_pt(self):
        return self.del_data.point

    def can_pt(self):
        return self.can_data.point


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

    def __init__(self, objective):
        State.__init__(self,
                       outcomes=['succeeded', 'lost', 'aborted'],
                       input_keys=['destination'],
                       output_keys=['initial_point']
                       )
        self.objective = objective
        self.destination = Point()

    def execute(self, userdata):
        global ms

        rospy.loginfo('Beginning Navigation to Destination')

        self.destination = userdata.destination
        client = SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('Waiting for MOVE_BASE SERVER ...')

        client.wait_for_server()
        rospy.loginfo('MOVE_BASE SERVER IS UP!')

        while True:
            goal = self.make_goal()
            client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5.0))  # wait 10 sec. until completion
            now = rospy.Time.now()

            ### FOR LATER, SET INITIAL POINT OF SEARCH ###
            if self.objective == 'discovery':
                t = ms.dis_data.time
                p = ms.dis_pt()
            elif self.objective == 'delivery':
                t = ms.del_data.time
                p = ms.del_pt()
            if p.x != 0 and p.y != 0:
                userdata.initial_point = [p.x, p.y, p.z]

            if (now - t).to_sec() > 5.0:
                # more than 5 seconds have passed since sight of can
                client.cancel_all_goals()
                return 'lost'  # lost can from sight somehow

            dist = self.destination.x ** 2 + self.destination.y ** 2
            if dist < 2.0:  # within 4 meters from can
                client.cancel_all_goals()  # start manual drive!
                return 'succeeded'

    def make_goal(self):
        """
        Calculates the position that the robot should go to in order to grab the can
        :return:
        """

        # TODO: prevent calculating positions too close to a wall
        # TODO: visualize this process better
        # TODO: publish goal in map frame to prevent errors if the robot moves in time
        global ms
        rospy.loginfo('Calculating navigation goal')

        if self.objective == 'discovery':
            dest = ms.dis_pt()
        elif self.objective == 'delivery':
            dest = ms.del_pt()

        # Transform can location into base_link
        pt = PointStamped(header=Header(stamp=rospy.Time(0), frame_id='map'), point=dest)
        self.destination = tf_listener.transformPoint("base_link", pt).point  # w.r.t self

        x, y = self.destination.x, self.destination.y
        theta = math.atan2(y, x)

        r = 1.0  # 1m back from target position
        x -= r * math.cos(theta)
        y -= r * math.sin(theta)

        angle = Quaternion(0, 0, math.sin(theta / 2), math.cos(theta / 2))

        dest = PoseStamped(
            header=Header(frame_id='base_link'),
            pose=Pose(position=Point(x=x, y=y, z=0), orientation=angle))

        goal = MoveBaseGoal(target_pose=dest)

        return goal


class Stuck(State):
    def __init__(self, n_attempts=10):
        State.__init__(
            self,
            outcomes=['succeeded', 'aborted']
        )
        self.initial_pose = Pose()
        self.n_attempts = n_attempts

    def execute(self, userdata):
        # TODO: Make it look less stupid when stuck
        while True:  # absolutely need to get the transforms.
            try:
                # TODO: make sure we can get a tf even with time lag
                t, r = tf_listener.lookupTransform('map', 'base_link', rospy.Time.now())
                break
            except:
                pass
        p = self.initial_pose.position
        o = self.initial_pose.orientation
        p.x, p.y, p.z = t
        o.x, o.y, o.z, o.w = r

        client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Waiting for MOVE_BASE SERVER ...')
        client.wait_for_server()
        rospy.loginfo('MOVE_BASE SERVER IS UP!')
        for i in range(self.n_attempts):
            if rospy.is_shutdown():
                exit()

            rospy.loginfo("Attempting Unstuck, {} / {}".format(i + 1, self.n_attempts))
            goal = self.make_goal()
            client.send_goal(goal)
            start = rospy.Time.now()
            while (rospy.Time.now() - start).to_sec() < 30.0:  # wait 30 sec. for success
                if rospy.is_shutdown():
                    exit()

                if client.wait_for_result(
                        rospy.Duration(1.0)):  # if move_base decides earlier that a goal is impossible...
                    res = client.get_state()
                    if res == 3:  ## SUCCEEDED
                        return 'succeeded'
        return 'aborted'

    def make_goal(self):
        dx = np.random.choice([np.random.uniform(-0.75, -0.25), np.random.uniform(0.25, 0.75)])
        dy = np.random.choice([np.random.uniform(-0.75, -0.25), np.random.uniform(0.25, 0.75)])

        p0 = self.initial_pose.position

        th = np.random.uniform(-np.pi, np.pi)
        qz = np.sin(th / 2)
        qw = np.cos(th / 2)

        target_pose = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                position=Point(p0.x + dx, p0.y + dy, p0.z),
                orientation=Quaternion(x=0, y=0, z=qz, w=qw)
            )
        )
        goal = MoveBaseGoal(target_pose=target_pose)
        return goal


class Explore(State):
    def __init__(self, objective):
        # boundary is a floating point number of square side length,
        # initial_point is a length-3 list
        State.__init__(self,
                       outcomes=['succeeded', 'discovered', 'aborted', 'stuck'],
                       input_keys=['boundary', 'initial_point'],
                       output_keys=['boundary', 'destination'])
        self.objective = objective
        self.last_mv = rospy.Time.now()

    def onCmdVel(self, msg):
        l = msg.linear
        a = msg.angular
        d = [l.x, l.y, l.z, a.x, a.y, a.z]
        eps = 1e-6
        for e in d:
            if abs(e) > eps:
                self.last_mv = rospy.Time.now()  # last movement

    def subscribe(self):
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.onCmdVel)

    def unsubscribe(self):
        if self.cmd_sub != None:
            self.cmd_sub.unregister()
            self.cmd_sub = None

    def execute_inner(self, userdata):
        global ms
        self.last_mv = rospy.Time.now()

        rospy.loginfo('Beginning Exploration')
        client = SimpleActionClient('explore_server', ExploreTaskAction)
        rospy.loginfo('WAITING FOR EXPLORE SERVER...')
        client.wait_for_server()
        rospy.loginfo('EXPLORE SERVER IS NOW UP!')

        boundary = PolygonStamped()
        boundary.header.frame_id = "map"
        boundary.header.stamp = rospy.Time.now()
        r = userdata.boundary / 2.0
        rospy.loginfo('boundary : {}'.format(r))
        x, y, _ = userdata.initial_point
        boundary.polygon.points.append(Point(x + r, y + r, 0))
        boundary.polygon.points.append(Point(x - r, y + r, 0))
        boundary.polygon.points.append(Point(x - r, y - r, 0))
        boundary.polygon.points.append(Point(x + r, y - r, 0))

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
            if rospy.is_shutdown():
                exit()

            if client.wait_for_result(rospy.Duration(0.3)):  # 0.3 sec. timeout to check for cans
                # The exploration node has finished
                res = client.get_state()
                rospy.loginfo('EXPLORE SERVER STATE:{}'.format(res))
                if res == 3:  ## SUCCEEDED
                    # if exploration is complete...
                    userdata.boundary += 1.0  # explore a larger area
                    return 'succeeded'  # finished! yay!
                else:
                    print 'explore server failed : {}'.format(res)
                    # when explore server gives up, can't explore
                    return 'stuck'

            now = rospy.Time.now()

            if self.objective == 'discovery':
                t = ms.dis_data.time
                p = ms.dis_pt()
            elif self.objective == 'delivery':
                t = ms.del_data.time
                p = ms.del_pt()
            else:
                rospy.logerr('Invalid objective passed to Explore state')
                return 'aborted'

            # if we're here, exploration is not complete yet...
            if t != None:  # check initialized
                dt = (now - t).to_sec()
                discovered = (dt < 2.0)  # last seen within the last 2 seconds
                if discovered:
                    # if can was found ...
                    client.cancel_all_goals()
                    userdata.destination = p
                    return 'discovered'

            # more than 20 seconds have passed while completely still
            # we're probably stuck
            if (now - self.last_mv).to_sec() > 20.0:
                print 'haven\'t been moving for a while!'
                client.cancel_all_goals()
                return 'stuck'  # bad name... "stuck" would be better

    def execute(self, userdata):
        self.subscribe()

        # select lidar data, depending on can
        rospy.loginfo('Selecting scan data ... ')

        rospy.wait_for_service('scan_select')
        scan_select = rospy.ServiceProxy('scan_select', MuxSelect)
        try:
            topic = 'scan_raw' if self.objective == 'discovery' else 'scan_filtered'
            mux_res = scan_select(topic=topic)
        except rospy.ServiceException as e:
            print 'Failed to Select Scan : ' + str(e)
            return 'aborted'

        rospy.loginfo('Successfully Selected scan data!')

        res = self.execute_inner(userdata)
        self.unsubscribe()
        return res


def Grip(close=True):
    gripper_goal = GripGoal()
    gripper_goal.do_grip = close
    return SimpleActionState('alpha_grip', GripAction, goal=gripper_goal)


class ProximityNav(State):
    # TODO: make this faster
    # TODO: go the right distance to actually grab the can
    def __init__(self, time=10, speed=0.2, kp=2.0, objective='discovery'):
        State.__init__(self, outcomes=['succeeded', 'lost'])
        self.timeout = rospy.Duration.from_sec(time)
        self.max_speed = speed
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angleError = 0
        self.kp = kp
        self.objective = objective

    def execute(self, userdata):
        global ms

        start_time = rospy.Time.now()
        rospy.loginfo('Beginning drive to can')
        r = rospy.Rate(20)

        # Drive the robot
        while rospy.Time.now() - start_time < self.timeout and not rospy.is_shutdown():

            now = rospy.Time.now()

            if self.objective == 'discovery':
                dest = ms.can_pt()  # look for cans, NOT april tags!
                t = ms.can_data.time
            elif self.objective == 'delivery':
                dest = ms.del_pt()  # here, you can look for april tags
                t = ms.del_data.time
            else:
                rospy.logerr('Invalid objective to ProximityNav')
                return 'lost'

            map_pt = PointStamped(header=Header(stamp=rospy.Time(0), frame_id='map'), point=dest)
            local_point = tf_listener.transformPoint('base_link', map_pt).point

            dist = math.sqrt(local_point.x ** 2 + local_point.y ** 2)
            angle_error = math.atan2(local_point.y, local_point.x)

            rospy.loginfo('Angle Error : {}; Distance : {}'.format(angle_error, dist))

            speed = 0.2 * dist + 0.1  # assumedly, given dist<1.0, always under approx. 0.2m/s
            if speed > self.max_speed:
                speed = self.max_speed

            turn_power = self.kp * angle_error
            self.pub.publish(Twist(linear=Vector3(x=speed), angular=Vector3(z=turn_power)))

            if (now - t).to_sec() > 1.0:
                # I lost the can 1 second ago
                if dist < 0.5:  # most likely, the can is too close to the robot so the camera cannot see
                    return 'succeeded'
                else:
                    return 'lost'

            if dist < 0.2:
                # I'm on top of the can
                return 'succeeded'

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


# main
def main():
    global tf_listener, ms

    ### INITIALIZE ROS ###
    rospy.init_node('alphabot_state_machine')

    ### INITIALIZING :: LOOKUP TF ###
    rospy.loginfo('Waiting for map->base_link tf transform ...')
    tf_listener = tf.TransformListener()
    t, r = None, None

    while True:
        try:
            now = rospy.Time.now()
            tf_listener.waitForTransform('map', 'base_link', now, rospy.Duration(4.0))
            t, r = tf_listener.lookupTransform('map', 'base_link', now)
            break
        except (tf.Exception) as e:
            print e

    rospy.loginfo('Successfully initialized tf transform!')

    ### SETUP SUBSCRIBERS ###
    ms = MissionSubscriber()

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted'],
                      input_keys=['destination', 'initial_point', 'boundary'])

    # Open the container
    with sm:

        # Add states to the container
        sm_dis = StateMachine(
            outcomes=['succeeded', 'aborted'],
            input_keys=['destination', 'initial_point', 'boundary'])

        with sm_dis:
            StateMachine.add('EXPLORE', Explore('discovery'),
                             transitions={
                                 'succeeded': 'EXPLORE',
                                 'stuck': 'STUCK',
                                 'discovered': 'NAV',
                                 'aborted': 'aborted'
                             }
                             )
            StateMachine.add('NAV', Navigate('discovery'),
                             transitions={
                                 'succeeded': 'RELEASE',  # release gripper before pnav
                                 'lost': 'EXPLORE',
                                 'aborted': 'EXPLORE'
                             }
                             )
            StateMachine.add('STUCK', Stuck(),
                             transitions={
                                 'succeeded': 'EXPLORE',
                                 'aborted': 'aborted'
                             }
                             )
            StateMachine.add('DELAY', Delay(3),
                             transitions={
                                 'succeeded': 'PNAV'
                             }
                             )
            StateMachine.add('PNAV', ProximityNav(objective='discovery'),
                             transitions={
                                 'succeeded': 'GRIP',
                                 'lost': 'EXPLORE'
                             }
                             )
            StateMachine.add('GRIP', Grip(True),
                             transitions={
                                 # Change to "succeeded" when not testing just PNAV
                                 'succeeded': 'aborted',  # start delivery
                                 'preempted': 'GRIP',
                                 'aborted': 'RELEASE'  # release before continuing nav
                             }
                             )
            StateMachine.add('RELEASE', Grip(False),
                             # TODO: try backing up before attempting to re-grip
                             transitions={
                                 'succeeded': 'DELAY',
                                 'preempted': 'RELEASE',
                                 'aborted': 'aborted'
                             }
                             )

        # TODO: Remove when not testing proximity nav
        sm_dis.set_initial_state(['RELEASE'])

        sm_del = StateMachine(
            outcomes=['succeeded', 'aborted'],
            input_keys=['destination', 'initial_point', 'boundary'])

        with sm_del:
            StateMachine.add('EXPLORE', Explore('delivery'),
                             transitions={
                                 'succeeded': 'EXPLORE',
                                 'stuck': 'STUCK',
                                 'discovered': 'NAV',
                                 'aborted': 'aborted'
                             }
                             )
            StateMachine.add('STUCK', Stuck(),
                             transitions={
                                 'succeeded': 'EXPLORE',
                                 'aborted': 'aborted'
                             }
                             )
            StateMachine.add('NAV', Navigate('delivery'),
                             transitions={
                                 'succeeded': 'RELEASE',
                                 'lost': 'EXPLORE',
                                 'aborted': 'EXPLORE'
                             }
                             )
            # StateMachine.add('PNAV',ProximityNav(objective='delivery'),
            #        transitions={
            #            'succeeded':'RELEASE',
            #            'lost':'EXPLORE'
            #            }
            #        )
            StateMachine.add('RELEASE', Grip(False),
                             transitions={
                                 'succeeded': 'succeeded',
                                 'preempted': 'aborted'
                             }
                             )

        StateMachine.add('DISCOVERY', sm_dis,
                         transitions={
                             'succeeded': 'DELIVERY',
                             'aborted': 'aborted'  # alternatively, halt
                         }
                         )

        StateMachine.add('DELIVERY', sm_del,
                         transitions={
                             'succeeded': 'succeeded',
                             'aborted': 'DELIVERY'
                         }
                         )
        sm.set_initial_state(['DISCOVERY'])

    # Execute the machine
    data = UserData()

    data.destination = Point(1, 0, 0)  # set to something ...
    data.boundary = 3.0  # start out with 3m x 3m boundary exploration for can
    data.initial_point = t

    sis = IntrospectionServer('smach', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute(data)

    print 'done_1'
    rospy.spin()
    print 'done_2'
    sis.stop()
    print 'done_3'


if __name__ == '__main__':
    main()
