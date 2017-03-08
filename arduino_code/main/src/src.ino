#include <ros.h>
ros::NodeHandle nh;

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "encoders.h"
#include "distanceSensors.h"
#include "motor.h"

const int MOTOR_L_PIN = 2;
const int MOTOR_R_PIN = 3;
const int E_STOP_PIN = 7;

// setup Parameters for the motor
// without these defined, it won't compile!
int Motor::STOP_SPEED = 1500;
int Motor::MIN_SPEED = 1250;
int Motor::MAX_SPEED = 1750;
int Motor::DELTA_SPEED = 5;

Motor motor_l;
Motor motor_r;

const int GRIPPER_PIN = 11;
const int G_GRIP = 180;
const int G_RELEASE = 0;
Servo gripper;
bool grip_status = G_RELEASE;

float v2p(float v){
	return 1523 + 408*v - 220*v*v; // based on calibration data
}

void vel_cb(const geometry_msgs::Twist& msg){

	float v = msg.linear.x;
	float w = msg.angular.z;
	const float l = 1.016; // base width

	float v_l = v - (w*l)/2;
	float v_r = v + (w*l)/2;

	motor_l.set_dst(v2p(v_l));
	motor_r.set_dst(v2p(v_r));
}

void grip_cb(const std_msgs::Bool& msg){
	grip_status = msg.data;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", vel_cb);
ros::Subscriber<std_msgs::Bool> g_sub("grip", grip_cb);

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

	//Gripper
	pinMode(GRIPPER_PIN, OUTPUT);
	gripper.attach(GRIPPER_PIN);

	nh.initNode();

	nh.subscribe(sub);
	nh.subscribe(g_sub);

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

	gripper.write(grip_status?G_GRIP:G_RELEASE);

	loopEncoders(motor_r.cur);
	loopDistanceSensors();
	delay(5);
}
