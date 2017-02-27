#include <ros.h>
ros::NodeHandle nh;

#include <geometry_msgs/Twist.h>
#include "encoders.h"
#include "distanceSensors.h"
#include "motor.h"

const int MOTOR_L_PIN = 2;
const int MOTOR_R_PIN = 3;
const int E_STOP_PIN = 7;

// setup Parameters for the motor
// without these defined, it won't compile!
int Motor::STOP_SPEED = 1500;
int Motor::MIN_SPEED = 1100;
int Motor::MAX_SPEED = 1900;
int Motor::DELTA_SPEED = 5;

Motor motor_l;
Motor motor_r;

void vel_cb(const geometry_msgs::Twist& msg){

	float l = msg.linear.x;
	float a = msg.angular.z;

	motor_l.set_dst( Motor::STOP_SPEED + 200*l - 200*a);
	motor_r.set_dst( Motor::STOP_SPEED + 200*l + 200*a);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", vel_cb);

bool readEstop(){
	return digitalRead(E_STOP_PIN);
}

void setup()
{
	//Serial.begin(9600);
	pinMode(E_STOP_PIN, INPUT);

	motor_l.attach(MOTOR_L_PIN);
	motor_r.attach(MOTOR_R_PIN);

	motor_l.stop();
	motor_r.stop();

	nh.initNode();
	nh.subscribe(sub);

	setupEncoders();
	setupDistanceSensors("l_ir","r_ir");
}

void loop()
{
	nh.spinOnce(); // vel_cb gets called here

	motor_l.ramp();
	motor_r.ramp();
	motor_l.write();
	motor_r.write();

	loopEncoders(motor_r.cur);
	loopDistanceSensors();
}
