#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include "encoders.h"
#include "distanceSensors.h"
//#include "motor.h"
#include "RoboClaw.h"
#include <Servo.h> // needs to be included before gipper.h
#include "gripper.h"
#include "PID_v1.h"

#define WHEEL_BASE 0.44
#define E_STOP_PIN 7

/* ===== SETUP ROS ===== */
ros::NodeHandle nh;
const int ODOM_LOOP_PERIOD = 20;
const int DISTANCE_LOOP_PERIOD = 100;
const int GRIPPER_LOOP_PERIOD= 50;
/* ===================== */

/* ===== SETUP ROBOCLAW ===== */
RoboClaw roboclaw(&Serial2, 10000); 
#define R_ADDR 0x80
/* ======================= */

/* ===== SETUP GRIPPER ===== */
#define GRIPPER_PIN 11
#define LIM_SW_PIN 22
int Gripper::G_GRIP = 75;
int Gripper::G_RELEASE = 20;
Gripper gripper(GRIPPER_PIN, LIM_SW_PIN, GRIPPER_LOOP_PERIOD);
/* ========================= */

/* ===== SETUP PID ===== */
double l_out, l_set;
double r_out, r_set;

#ifdef TUNE_PID
float K_P=1.0, K_I=0.0, K_D=0.0;
#else
#define K_P 0.75
#define K_I 0.05
#define K_D 0.0
#endif


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
	return 1.18*128*v; // for RoboClaw
	// actually pretty close to commanded value!
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

#ifdef TUNE_PID
void pid_cb(const geometry_msgs::Point& msg){
	K_P = msg.x;
	K_I = msg.y;
	K_D = msg.z;

	resetPIDs();
	l_pid.SetTunings(K_P,K_I,K_D);
	r_pid.SetTunings(K_P,K_I,K_D);
}
ros::Subscriber<geometry_msgs::Point> pid_sub("pid", pid_cb);
#endif

bool readEstop(){
	return digitalRead(E_STOP_PIN);
}

void setup()
{
	//Serial.begin(9600);
	pinMode(E_STOP_PIN, INPUT);

	nh.initNode();
	nh.subscribe(sub);
#ifdef TUNE_PID
	nh.subscribe(pid_sub);
#endif

	setupEncoders(nh);
	setupDistanceSensors(nh, "l_ir","r_ir","ultrasound");
	gripper.setup(nh);

	l_pid.SetMode(AUTOMATIC);
	r_pid.SetMode(AUTOMATIC);

	roboclaw.begin(38400);
}

void loop()
{
	nh.spinOnce(); // vel_cb gets called here

	// run PID
	if(r_pid.Compute()){
		if(r_set > 0){
			roboclaw.ForwardM1(R_ADDR, v2p(r_set));
		}else{
			roboclaw.BackwardM1(R_ADDR, v2p(-r_set));
		}
	}
	if(l_pid.Compute()){
		if(l_set > 0){
			roboclaw.ForwardM2(R_ADDR, v2p(l_set));
		}else{
			roboclaw.BackwardM2(R_ADDR, v2p(-l_set));
		}
	}

	loopEncoders();
	loopDistanceSensors(nh); // 100 m period
	gripper.loop(); // 50 ms period
}
