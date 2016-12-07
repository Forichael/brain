#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

const float WHEEL_SEP = .441; //in m

float pwm2rpm(float pwm){
	return (pwm - 1500) * 1.25;
}

float rpm2vel(float rpm){
	// in m/s
	return rpm / 60.0 * (M_PI * .131);
}

float pwm2vel(float pwm){
	return rpm2vel(pwm2rpm(pwm));
}

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg){

	ros::Time now = ros::Time::now();

	float ang_pwm = msg->angular.z * 200;
	float lin_pwm = msg->linear.x * 200;

	float v_l = pwm2vel(lin_pwm - ang_pwm);
	float v_r = pwm2vel(lin_pwm + ang_pwm);

	//float r = (WHEEL_SEP/2) * (v_r + v_l) / (v_r - v_l); // radius of icc
	float w = (v_r - v_l) / WHEEL_SEP;
	float v = 

}

void time_cb(

int main(){

}
