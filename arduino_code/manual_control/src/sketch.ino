#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

const int MOTOR_L_PIN = 2;
const int MOTOR_R_PIN = 3;
const int E_STOP_PIN = 7;

const int STOP = 1500; // linear stop

const int MIN = 1300;
const int MAX = 1700;

const int DELTA_SPEED = 5;

int speed_l_dst = 0.0;
int speed_r_dst = 0.0;

ros::NodeHandle nh;

struct Motor{
	Servo motor;

	int pin;
	int cur; // current speed
	int dst; // target speed

	Motor(){

	}

	void ramp(){
		if(cur < dst){
			cur += DELTA_SPEED;
		}else if (cur > dst){
			cur -= DELTA_SPEED;
		}
	}

	void attach(const int pin){
		this->pin = pin;
		motor.attach(pin);
	}

	void set_dst(int val){
		//protect against too much
		if (val > MAX){
			dst = MAX;
		}else if (val < MIN){
			dst = MIN;
		}else{
			dst = val;
		}
	}
	void set_cur(int val){
		cur = val;
	}
	void stop(){
		set_dst(STOP);
		set_cur(STOP);
		write();
	}
	void write(){
		motor.writeMicroseconds(cur);
	}
};

Motor motor_l;
Motor motor_r;

void vel_cb(const geometry_msgs::Twist& msg){

	float l = msg.linear.x;
	float a = msg.angular.z;

	motor_l.set_dst( STOP + 200*l - 200*a);
	motor_r.set_dst( STOP + 200*l + 200*a);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", vel_cb);

bool readEstop(){
	return digitalRead(E_STOP_PIN);
}

void setup()
{
	//Serial.begin(9600);
	pinMode(MOTOR_L_PIN, OUTPUT);
	pinMode(MOTOR_R_PIN, OUTPUT);
	pinMode(E_STOP_PIN, INPUT);

	motor_l.attach(MOTOR_L_PIN);
	motor_r.attach(MOTOR_R_PIN);

	motor_l.stop();
	motor_r.stop();

	nh.subscribe(sub);
	nh.initNode();
}

void loop()
{
	nh.spinOnce(); // vel_cb gets called here

	motor_l.ramp();
	motor_r.ramp();
	motor_l.write();
	motor_r.write();
}
