//
// Created by eric on 1/29/17.
//
#include <ros.h>
#include <Encoder.h>
#include <Arduino.h>
#include <std_msgs/Int16.h>

#ifndef MAIN_ODOMETRY_H
#define MAIN_ODOMETRY_H
extern ros::NodeHandle nh;
// Setup encoder data
Encoder rightEncoder(18, 20);
Encoder leftEncoder(19, 21);

const int ODOM_LOOP_TIME = 5; // ms

std_msgs::Int16 right_encoder_msg;
ros::Publisher right_encoder_pub("/rwheel", &right_encoder_msg);

std_msgs::Int16 left_encoder_msg;
ros::Publisher left_encoder_pub("/lwheel", &left_encoder_msg);


void updateEncoders(){
    right_encoder_msg.data = rightEncoder.read();
    right_encoder_pub.publish( &right_encoder_msg );

    left_encoder_msg.data = leftEncoder.read();
    left_encoder_pub.publish( &left_encoder_msg );
}

void setupEncoders(){
    nh.advertise(right_encoder_pub);
    nh.advertise(left_encoder_pub);
}

void loopEncoders(float right_dir){
    static unsigned long lastOdomTime = millis();

    if (millis() - lastOdomTime >= ODOM_LOOP_TIME) {
        lastOdomTime = millis();
        updateEncoders();
    }
}


#endif //MAIN_ODOMETRY_H
