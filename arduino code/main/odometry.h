//
// Created by eric on 1/29/17.
//
#include <Encoder.h>
#include <Arduino.h>

#ifndef MAIN_ODOMETRY_H
#define MAIN_ODOMETRY_H


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder rightEncoder(20,21);

const unsigned long ENCODER_TIMEOUT = 500*long(1000); // microseconds, speed is zero if no reading in this time.
const long encoderSmoothingTime = 100*long(1000); //microseconds


long lastEncoderPosition = 0;
long lastEncoderDeltaPosition = 0;
unsigned long lastEncoderChangeTime = 0;
unsigned long lastEncoderDeltaTime = 1; // microseconds


// Returns position in ticks
float getEncoderVal(){
    return rightEncoder.read();
}

// Returns speed in ticks / second
float getEncoderSpeed(){
    unsigned long time = micros();
    if (time - lastEncoderChangeTime >= ENCODER_TIMEOUT)
    {
        return 0.0;
    }

    return (float(lastEncoderDeltaPosition) / lastEncoderDeltaTime) * 1e6;
}

void loopEncoder(){
    long val = rightEncoder.read();
    if (val != lastEncoderPosition)
    {
        unsigned long time = micros();
        if (time - lastEncoderChangeTime >= encoderSmoothingTime)
        {
            lastEncoderDeltaTime = time - lastEncoderChangeTime;
            lastEncoderDeltaPosition = val - lastEncoderPosition;

            lastEncoderChangeTime = time;
            lastEncoderPosition = val;
        }
    }
}

bool printIfNewEncoder(){
    static long oldPosition  = -999;


    long newPosition = rightEncoder.read();
    if (newPosition != oldPosition) {
        oldPosition = newPosition;
        Serial.println(newPosition);
        return true;
    }
    return false;
}

#endif //MAIN_ODOMETRY_H
