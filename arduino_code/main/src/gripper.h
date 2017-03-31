#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <Servo.h>
class Gripper{

public:
	static int G_GRIP;
	static int G_RELEASE; //require to define these
private:
	const int g_pin, l_pin, loop_period;
	Servo gripper;
	ros::Subscriber<std_msgs::Bool,Gripper> grip_sub;
	std_msgs::Bool lim_sw_msg;
	ros::Publisher lim_sw_pub;

	unsigned long last_debounce_time;
	bool last_sw_state;

	unsigned long last_loop_time;

public:
	Gripper(const int g_pin, const int l_pin, const int loop_period):
		g_pin(g_pin),
		l_pin(l_pin),
		loop_period(loop_period),
		grip_sub("/grip", &Gripper::grip_cb,(Gripper*)this),
		lim_sw_pub("/lim_sw", &lim_sw_msg){
	}

	void setup(ros::NodeHandle& nh){
		pinMode(g_pin, OUTPUT);
		pinMode(l_pin, INPUT);
		gripper.attach(g_pin);
		last_loop_time = millis();
		last_debounce_time = millis();
		last_sw_state = digitalRead(l_pin);

		grip(false);
		nh.subscribe(grip_sub);
		nh.advertise(lim_sw_pub);
	}

	void grip_cb(const std_msgs::Bool& msg){
		grip(msg.data);
	}

	void grip(bool close){
		gripper.write(close?G_GRIP:G_RELEASE);	
	}

	void loop(){
		unsigned long now = millis();

		/* ===== DEBOUNCING ===== */
		bool lim_status = digitalRead(l_pin); 
		if(lim_status != last_sw_state){
			// change occurred
			last_debounce_time = now;
		}

		if(now - last_debounce_time > 10){ // 10 ms debounce delay
			lim_sw_msg.data = lim_status;
		}
		/* ======================= */

		// publish
		if(now - last_loop_time > loop_period){
			lim_sw_pub.publish(&lim_sw_msg);
			last_loop_time = now;
		}

		last_sw_state = lim_status;
	}
};

#endif
