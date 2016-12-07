#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

const float WHEEL_SEP = .441; //in m
const float WHEEL_DIAM = .131;

ros::Time prev;

geometry_msgs::TwistWithCovarianceStamped twist_msg;

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

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg){

	ros::Time now = ros::Time::now();

	float ang_pwm = msg->angular.z * 200;
	float lin_pwm = msg->linear.x * 200;

	float v_l = pwm2vel(lin_pwm - ang_pwm);
	float v_r = pwm2vel(lin_pwm + ang_pwm);


	float w = (v_r - v_l) / WHEEL_SEP; // angular velocity

	float r = (WHEEL_SEP/2) * (v_r + v_l) / (v_r - v_l); // radius of icc

	float v = w * r;  // forward velocity

	twist_msg.twist.twist.angular.z = w;
	twist_msg.twist.twist.linear.x = v;
	//twist_msg.twist.covariance
}

void time_cb(){

}
int main(int argc, char* argv[]){
	ros::init(argc,argv,"wheel_odometry");
	ros::NodeHandle n;

	prev = ros::Time::now();

	ros::Subscriber imuSub = n.subscribe("/cmd_vel", 1000, cmd_vel_cb);

	ros::spin();
	return 0;
}
