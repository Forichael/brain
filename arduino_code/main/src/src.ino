#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include "encoders.h"
#include "distanceSensors.h"
#include "motor.h"
#include "gripper.h"
#include "PID_v1.h"

#define WHEEL_BASE 0.44

/* ===== SETUP ROS ===== */
ros::NodeHandle nh;
const int ODOM_LOOP_PERIOD = 20;
const int DISTANCE_LOOP_PERIOD = 100;
const int GRIPPER_LOOP_PERIOD= 50;
/* ===================== */

/* ===== SETUP MOTOR ===== */
#define MOTOR_L_PIN 2
#define MOTOR_R_PIN 3
#define E_STOP_PIN 7
int Motor::STOP_SPEED = 1520;
int Motor::MIN_SPEED = STOP_SPEED - 250;
int Motor::MAX_SPEED = STOP_SPEED + 250;
int Motor::DELTA_SPEED = 5;
Motor motor_l;
Motor motor_r;
/* ======================= */

/* ===== SETUP GRIPPER ===== */
#define GRIPPER_PIN 11
#define LIM_SW_PIN 22
int Gripper::G_GRIP = 180;
int Gripper::G_RELEASE = 90;
Gripper gripper(GRIPPER_PIN, LIM_SW_PIN, GRIPPER_LOOP_PERIOD);
/* ========================= */

/* ===== SETUP PID ===== */
double l_out, l_set;
double r_out, r_set;

// #define K_P 0.8775
#define K_P 0.6
#define K_I 0.173
#define K_D 0.0

//l_vel and r_vel are computed from encoders.h, in loopEncoders()
PID l_pid(&l_vel, &l_out, &l_set, K_P, K_I, K_D, DIRECT); //TODO : tune k_p, k_i, k_d
PID r_pid(&r_vel, &r_out, &r_set, K_P, K_I, K_D, DIRECT);
/* ====================== */

void resetPIDs(){
	l_pid.SetMode(MANUAL);
	r_pid.SetMode(MANUAL);
	l_out = r_out = 0;
	l_pid.SetMode(AUTOMATIC);
	r_pid.SetMode(AUTOMATIC);
}

float v2p(float v){
	return Motor::STOP_SPEED + 408*v - 220*v*v;
	//return Motor::STOP_SPEED + 408 * v; // made linear
}

void vel_cb(const geometry_msgs::Twist& msg){
	float v = msg.linear.x;
	float w = msg.angular.z;

	// setpoints based on cmd_vel
	l_set = v - (w*WHEEL_BASE);
	r_set = v + (w*WHEEL_BASE);

	if (v == 0 && w == 0){
		resetPIDs();
	}
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

	setupEncoders(nh);
	setupDistanceSensors(nh, "l_ir","r_ir");
	gripper.setup(nh);

	l_pid.SetMode(AUTOMATIC);
	r_pid.SetMode(AUTOMATIC);
}

void loop()
{
	nh.spinOnce(); // vel_cb gets called here
	// run PID
	if(l_pid.Compute())
		motor_l.set_dst(v2p(l_out));
	if(r_pid.Compute())
		motor_r.set_dst(v2p(r_out));

	motor_l.ramp();
	motor_r.ramp();
	motor_l.write();
	motor_r.write();

	loopEncoders();
	loopDistanceSensors(nh); // 100 m period
	gripper.loop(); // 50 ms period
}
