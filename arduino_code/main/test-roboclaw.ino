#include "RoboClaw.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


RoboClaw roboclaw(&Serial2, 10000);
#define ROBOCLAW_ADDRESS 0x80
#define WHEEL_BASE 0.44


ros::NodeHandle nh;

int l_speed = 0;
int r_speed = 0;

void vel_cb(const geometry_msgs::Twist& msg){
	float v = msg.linear.x;
	float w = msg.angular.z;

	// setpoints based on cmd_vel
	float l = v - (w*WHEEL_BASE);
	float r = v + (w*WHEEL_BASE);

	l_speed = l * 128;
	r_speed = r * 128;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",vel_cb);


void setup(){
	nh.initNode();
	nh.subscribe(sub);
	roboclaw.begin(38400);
}

void loop(){
// r = m1, l = m2
	if(r_speed > 0){
		roboclaw.ForwardM1(ROBOCLAW_ADDRESS,r_speed);
	}else{
		roboclaw.BackwardM1(ROBOCLAW_ADDRESS,-r_speed);
	}

	if(l_speed > 0){
		roboclaw.ForwardM2(ROBOCLAW_ADDRESS,l_speed);
	}else{
		roboclaw.BackwardM2(ROBOCLAW_ADDRESS,-l_speed);
	}

	nh.spinOnce();
}
