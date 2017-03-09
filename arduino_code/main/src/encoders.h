//
// Created by eric on 1/29/17.
//
#include <ros.h>
#include <Encoder.h>
#include <Arduino.h>
#include <std_msgs/Int16.h>

#ifndef MAIN_ODOMETRY_H
#define MAIN_ODOMETRY_H

#define TICKS_PER_METER 6261

extern ros::NodeHandle nh;
// Setup encoder data

int32_t l_prv_pos;
int32_t r_prv_pos;

Encoder rightEncoder(18, 20);
Encoder leftEncoder(19, 21);

double l_vel;
double r_vel;

const int ODOM_LOOP_TIME = 20; // ms

std_msgs::Int16 right_encoder_msg;
ros::Publisher right_encoder_pub("/rwheel", &right_encoder_msg);

std_msgs::Int16 left_encoder_msg;
ros::Publisher left_encoder_pub("/lwheel", &left_encoder_msg);

std_msgs::Int16 wtf_msg;
ros::Publisher wtf_pub("/wtf", &wtf_msg);

void updateEncoders(float dt){
	int16_t l_pos = leftEncoder.read();
	int16_t r_pos = rightEncoder.read();

    right_encoder_msg.data = r_pos;
    right_encoder_pub.publish( &right_encoder_msg );

    left_encoder_msg.data = l_pos;
    left_encoder_pub.publish( &left_encoder_msg );

	int32_t dl = (l_pos - l_prv_pos);
	int32_t dr = (r_pos - r_prv_pos);

	if(dt > 0){
		// convert to m/s

		// deal with wrap-arounds
		if(dl < -32768) dl += 65536;
		if(dl > 32768) dl -= 65536;
		if(dr < -32768) dr += 65536;
		if(dr > 32768) dr -= 65536;

		l_vel = (dl/dt) / TICKS_PER_METER;
		r_vel = (dl/dt) / TICKS_PER_METER;
	}

	wtf_msg.data = dl;
	wtf_pub.publish(&wtf_msg);

	l_prv_pos = l_pos;
	r_prv_pos = r_pos;
}

void setupEncoders(){
    nh.advertise(right_encoder_pub);
    nh.advertise(left_encoder_pub);
	nh.advertise(wtf_pub);
}

void loopEncoders(){
    static unsigned long lastOdomTime = millis();
	unsigned long now = millis();
	unsigned long dt = now - lastOdomTime;
    if (dt >= ODOM_LOOP_TIME) {
        updateEncoders(dt/1000.); // convert dt to seconds
        lastOdomTime = now;
    }
}

#endif //MAIN_ODOMETRY_H
