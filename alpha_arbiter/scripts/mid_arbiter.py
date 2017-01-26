#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

ARRAY_SIZE = 11
INPUTS = ['wpt', 'obst', 'wall']

class Midbrain_Arbiter(object):
	def __init__(self):
		rospy.init_node('midbrain_arbiter')

		self.vel_array = np.zeros([len(INPUTS), ARRAY_SIZE])
		self.vel_array[:,5] = 0
		self.turn_array = np.zeros([len(INPUTS), ARRAY_SIZE])
		self.turn_array[:,5] = 0
		self.initial_turn_array = [0.005, 0.006, 0.007, 0.008, 0.009, 0.01, 0.009, 0.008, 0.007, 0.006, 0.005]

		rospy.Subscriber('wpt/cmd_vel', Float32MultiArray, self.wpt_cmd_vel_cb)
		rospy.Subscriber('obst/cmd_vel', Float32MultiArray, self.obst_cmd_vel_cb)
		rospy.Subscriber('wall/cmd_vel', Float32MultiArray, self.wall_cmd_vel_cb)

		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def wpt_cmd_vel_cb(self, msg):
		self.update_array(msg.data, INPUTS.index('wpt'))

	def obst_cmd_vel_cb(self, msg):
		self.update_array(msg.data, INPUTS.index('obst'))

	def wall_cmd_vel_cb(self, msg):
		self.update_array(msg.data, INPUTS.index('wall'))

	def update_array(self, data, row):
		data = np.asarray(data).reshape([2, ARRAY_SIZE])
		self.vel_array[row] = data[0]
		self.turn_array[row] = data[1]

	def run(self):
		vel_sum_array = np.zeros(ARRAY_SIZE)
		turn_sum_array = np.zeros(ARRAY_SIZE)
		#vel_sum_array = np.array([1., 2, 3, 4, 5, 6, 5, 5 ,5, 2])
		#turn_sum_array = np.array([0., 0, 10, 1, 2, 3, 3, 2, 1, 1])
		turn_sum_array += self.initial_turn_array

		for i in range(len(INPUTS)):
			vel_sum_array += self.vel_array[i]
			turn_sum_array += self.turn_array[i]

		vel = vel_sum_array.argmax()
		turn = turn_sum_array.argmax()
		msg = Twist()
		msg.linear.x = (2.0*vel/(ARRAY_SIZE-1)-1)/2.0
		msg.angular.z = 2.0*turn/(ARRAY_SIZE-1)-1
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Midbrain_Arbiter()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()