#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <iostream>

const float WHEEL_SEP = .441; //in m
const float WHEEL_DIAM = .131;

ros::Time prev;

geometry_msgs::TwistWithCovarianceStamped twist_msg;

ros::Publisher twist_pub;

float pwm2rpm(float pwm){
	//conversion with motors
	return (pwm - 1500) * 1.25;
}

float rpm2vel(float rpm){
	//conversion based on wheel diameter
	// in m/s
	return rpm / 60.0 * (M_PI * WHEEL_DIAM);
}

float pwm2vel(float pwm){
	return rpm2vel(pwm2rpm(pwm));
}

float limit(float low, float x, float high){
	if(x < low)
		return low;
	else if (x > high)
		return high;
	return x;
}

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg){

	ros::Time now = ros::Time::now();

	float pwm_l = limit(1300, 1500 + 200 * msg->linear.x - 200 * msg->angular.z, 1700);
	float pwm_r = limit(1300, 1500 + 200 * msg->linear.x + 200 * msg->angular.z, 1700);

	float v_l = pwm2vel(pwm_l);
	float v_r = pwm2vel(pwm_r);

	float w,v;

	w = (v_r - v_l) / WHEEL_SEP;
	if(v_l == v_r){
		v = v_l;
	}else{
		float r = (WHEEL_SEP/2) * (v_r + v_l) / (v_r - v_l); // radius of icc
		v = r * w;  // forrard velocity
	}

	twist_msg.header.frame_id = "base_link";
	twist_msg.header.stamp = now;

	twist_msg.twist.twist.angular.z = w;
	twist_msg.twist.twist.linear.x = v;

	double* elems = twist_msg.twist.covariance.elems;

	// x, y, z, rx, ry, rz
	elems[0] = elems[7] = elems[14] = elems[21] = elems[28] = elems[35] = 1e-3;

	twist_pub.publish(twist_msg);
}

void time_cb(){

}
int main(int argc, char* argv[]){
	ros::init(argc,argv,"wheel_odometry");
	ros::NodeHandle n;

	prev = ros::Time::now();

	ros::Subscriber imuSub = n.subscribe("/cmd_vel", 1000, cmd_vel_cb);
	twist_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/wheel_twist", 1000, false);

	ros::spin();
	return 0;
}
