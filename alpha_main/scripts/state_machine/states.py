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

    def __init__(self, objective):
        State.__init__(self,
                outcomes=['succeeded','lost','aborted'],
                input_keys=['destination'],
                output_keys=['initial_point']
                )

        self.objective = objective

        self.destination = Point()

        self.dests_x = []
        self.dests_y = []

        self.dest_in_map = Point()
        self.last_update = rospy.Time.now().to_sec()

    def onMsg(self, msg):
        n_attempts = 10
        for _ in range(10):
            try:
                now = rospy.Time.now()
                tf_listener.waitForTransform('map','base_link', now, rospy.Duration(1.0))
                dest = tf_listener.transformPoint("base_link", msg)
                self.destination = dest.point
                p = tf_listener.transformPoint("map", msg).point
                self.dest_in_map = p

                #self.dests_x.append(p.x)
                #self.dests_y.append(p.y)

                #if len(self.dests_x) > 10:
                #    self.dest_in_map = filtered_can_point(self.dests_x, self.dests_y)
                #    f_p = PointStamped() 
                #    f_p.header.frame_id = "map"
                #    f_p.header.stamp = now 
                #    f_p.point = self.dest_in_map
                #    filtered_can_point_pub.publish(f_p)
                #    self.destination = tf_listener.transformPoint("base_link", f_p).point
                #else:
                #    self.dest_in_map = p

                self.last_update = now.to_sec()
                return
            except (tf.Exception, tf.ExtrapolationException) as e:
                pass

    def execute(self, userdata):

        if self.objective == 'discovery':
            self.sub = rospy.Subscriber('/dis_pt',PointStamped,self.onMsg)
        elif self.objective == 'delivery':
            self.sub = rospy.Subscriber('/del_pt',PointStamped,self.onMsg)
        else:
            return 'aborted'

        rospy.loginfo('Beginning Navigation to Destination')

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

                return 'lost' # lost can from sight somehow
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

class Stuck(State):
    def __init__(self, n_attempts=10):
        State.__init__(
                self,
                outcomes=['succeeded','aborted']
                )
        self.initial_pose = Pose()
        self.n_attempts = n_attempts

    def execute(self, userdata):
        while True: # absolutely need to get the transforms.
            try:
                t,r = tf_listener.lookupTransform('map','base_link', rospy.Time.now())
                break
            except:
                pass
        p = self.initial_pose.position
        o = self.initial_pose.orientation
        p.x,p.y,p.z = t
        o.x,o.y,o.z,o.w = r
        
        client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Waiting for MOVE_BASE SERVER ...')
        client.wait_for_server()
        rospy.loginfo('MOVE_BASE SERVER IS UP!')

        for _ in self.n_attempts:
            goal = self.make_goal()
            state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(20.0)) # wait 20 sec. until completion
            rospy.loginfo('Attempting Unstuck, State : {}'.format(state))
            if state == 3: # succeeded
                return 'succeeded'
        return 'aborted'

    def make_goal(self):
        r = 1.0
        dx,dy = r * np.random.random(2)
        p0 = self.initial_pose.position

        th = np.random.uniform(-np.pi,np.pi)
        qz = np.sin(th/2)
        qw = np.cos(th/2)

        target_pose = PoseStamped(
                header=Header(frame_id='map'),
                pose=Pose(
                    position=Point(p0.x+dx,p0.y+dy,p0.z),
                    orientation=Quaternion(x=0,y=0,z=qz,w=qw)
                    )
                )
        goal = MoveBaseGoal(target_pose=target_pose)
        return goal


class Explore(State):
    def __init__(self, objective):
        State.__init__(self,
                outcomes=['succeeded', 'discovered', 'aborted', 'stuck'],
                input_keys=['boundary', 'initial_point'],
                output_keys=['boundary', 'destination'])

        self.objective = objective
        self.dst_point = None
        self.dst_time = None 

        self.last_mv = rospy.Time.now()

        #SimpleActionState.__init__(
        #        self,
        #        'explore_server',
        #        ExploreTaskAction,
        #        goal_cb=self.goal_cb,
        #        )

    def onMsg(self, msg):
        self.dst_point = tf_listener.transformPoint("base_link", msg).point
        self.dst_time = msg.header.stamp

    def onCmdVel(self, msg):
        l = msg.linear
        a = msg.angular
        d = [l.x, l.y, l.z, a.x, a.y, a.z]
        eps = 1e-6

        for e in d:
            if abs(e) > eps:
                last_mv = rospy.Time.now() # last movement

    def subscribe(self):
        if self.objective == 'discovery':
            self.sub = rospy.Subscriber('/dis_pt',PointStamped,self.onMsg)
        elif self.objective == 'delivery':
            self.sub = rospy.Subscriber('/del_pt',PointStamped,self.onMsg)
        self.cmd_sub= rospy.Subscriber('/cmd_vel',Twist,self.onCmdVel)

    def unsubscribe(self):
        if self.sub != None:
            self.sub.unregister()
        if self.cmd_sub != None:
            self.cmd_sub.unregister()

    def execute_inner(self, userdata):
        self.last_mv = rospy.Time.now()

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
                    return 'succeeded' # finished! yay!
                elif res == 4: ## "ABORTED" ?? keep trying...
                    return 'succeeded'
                else:
                    # when explore server gives up, can't explore
                    return 'stuck'

            #if we're here, exploration is not complete yet...
            if self.dst_time != None: # check initialized
                discovered = (rospy.Time.now() - self.dst_time).to_sec() < 0.3
                if discovered:
                    # if can was found ...
                    client.cancel_all_goals()
                    userdata.destination = self.dst_point
                    return 'discovered'

            # more than 10 seconds have passed while completely still
            # we're probably stuck
            if (rospy.Time.now() - self.last_mv).to_sec() > 20.0:
                client.cancel_all_goals()
                return 'stuck' # bad name... "stuck" would be better
        return 'aborted'

    def execute(self, userdata):
        self.subscribe()
        res = self.execute_inner(userdata)
        self.unsubscribe()
        return res

        
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
    def __init__(self, time=6, speed=0.2, kp=2.0):
        State.__init__(self, outcomes=['succeeded','lost'])
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

        self.last_detected = rospy.Time.now()

        # Drive the robot
        while rospy.Time.now() - start_time < self.timeout and not rospy.is_shutdown():
            turnPower = self.kp * self.angleError
            self.pub.publish(Twist(linear=Vector3(x=self.speed), angular=Vector3(z=turnPower)))
            r.sleep()
            if (rospy.Time.now() - self.last_detected).to_sec() > 0.5:
                self.sub.unregister()
                return 'lost'

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
    global tf_listener
    rospy.init_node('alphabot_state_machine')
    rospy.loginfo('waiting for map->base_link tf transform ...')

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
            input_keys=['destination', 'initial_point', 'boundary'])

    # Open the container
    with sm:

        # Add states to the container
        sm_dis = StateMachine(
                outcomes=['succeeded', 'aborted'],
                input_keys=['destination','initial_point','boundary'])

        with sm_dis:
            StateMachine.add('EXPLORE', Explore('discovery'),
                    transitions={
                        'succeeded' : 'EXPLORE',
                        'stuck' : 'STUCK',
                        'discovered' : 'NAV',
                        'aborted' : 'aborted'
                        }
                    )
            StateMachine.add('NAV', Navigate('discovery'),
                    transitions={
                        'succeeded':'PNAV',
                        'lost':'EXPLORE',
                        'aborted':'EXPLORE'
                        }
                    )
            StateMachine.add('STUCK', Stuck('EXPLORE'),
                    transitions={
                        'succeeded':'EXPLORE',
                        'aborted':'aborted'
                        }
                    )
            StateMachine.add('PNAV',ProximityNav(speed=0.2),
                    transitions={
                        'succeeded':'GRIP',
                        'lost':'EXPLORE'
                        }
                    )
            StateMachine.add('GRIP', Grip(True),
                    transitions={
                        'succeeded': 'succeeded',
                        'preempted': 'GRIP',
                        'aborted': 'PNAV'
                        }
                    )

        sm_del = StateMachine(
                outcomes=['succeeded', 'aborted'],
                input_keys=['destination','initial_point','boundary'])

        with sm_del:
            StateMachine.add('EXPLORE', Explore('delivery'),
                    transitions={
                        'succeeded' : 'EXPLORE',
                        'stuck' : 'EXPLORE',
                        'discovered' : 'NAV',
                        'aborted' : 'aborted'
                        }
                    )
            StateMachine.add('NAV', Navigate('delivery'),
                    transitions={
                        'succeeded':'PNAV',
                        'lost':'EXPLORE',
                        'aborted':'EXPLORE'
                        }
                    )
            StateMachine.add('PNAV',ProximityNav(speed=0.2),
                    transitions={
                        'succeeded':'RELEASE',
                        'lost':'EXPLORE'
                        }
                    )
            StateMachine.add('RELEASE', Grip(False),
                    transitions={
                        'succeeded': 'succeeded',
                        'preempted': 'aborted'
                        }
                    )

        StateMachine.add('DISCOVERY', sm_dis,
                transitions={
                    'succeeded': 'DELIVERY',
                    'aborted':'DISCOVERY' # alternatively, halt
                    }
                )

        StateMachine.add('DELIVERY', sm_del,
                transitions={
                    'succeeded':'succeeded',
                    'aborted':'DELIVERY'
                    }
                )


        sm.set_initial_state(['DISCOVERY'])

    # Execute the machine
    data = UserData()

    data.destination = Point(1,0,0) #set to something ...
    data.boundary = 3.0 # start out with 3m x 3m boundary exploration for can
    data.initial_point = t

    sis = IntrospectionServer('smach', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute(data)

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
