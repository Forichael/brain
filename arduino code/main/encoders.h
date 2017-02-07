//
// Created by eric on 1/29/17.
//
#include <Encoder.h>
#include <Arduino.h>
#include <std_msgs/Int16.h>

#ifndef MAIN_ODOMETRY_H
#define MAIN_ODOMETRY_H

// Setup encoder data
Encoder rightEncoder(18, 19);

const int ODOM_LOOP_TIME = 20; // ms

std_msgs::Int16 right_encoder_msg;
ros::Publisher right_encoder_pub("/rwheel", &right_encoder_msg);


int hackEncoderVal = 0;

void updateEncoders(){
    right_encoder_msg.data = hackEncoderVal;
    //right_encoder_msg.data = rightEncoder.read();
    right_encoder_pub.publish( &right_encoder_msg );
}

void setupEncoders(){
    nh.advertise(right_encoder_pub);
}

void loopEncoders(float right_dir){
    static unsigned long lastOdomTime = millis();
    static int lastEncoderValue = rightEncoder.read();

    int value = rightEncoder.read();
    if (lastEncoderValue != value){
        if (right_dir > 0)
            hackEncoderVal += abs(value - lastEncoderValue);
        else
            hackEncoderVal -= abs(value - lastEncoderValue);

        lastEncoderValue = value;
    }

    if (millis() - lastOdomTime >= ODOM_LOOP_TIME) {
        lastOdomTime = millis();
        updateEncoders();
    }
}


#endif //MAIN_ODOMETRY_H
