#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Servo.h>

struct Motor{
	Servo motor;
	static int DELTA_SPEED, MAX_SPEED, MIN_SPEED, STOP_SPEED;

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
		pinMode(pin, OUTPUT);
		motor.attach(pin);
	}

	void set_dst(int val){
		//protect against too much
		if (val > MAX_SPEED){
			dst = MAX_SPEED;
		}else if (val < MIN_SPEED){
			dst = MIN_SPEED;
		}else{
			dst = val;
		}
	}
	void set_cur(int val){
		cur = val;
	}
	void stop(){
		set_dst(STOP_SPEED);
		set_cur(STOP_SPEED);
		write();
	}
	void write(){
		motor.writeMicroseconds(cur);
	}
};

#endif
