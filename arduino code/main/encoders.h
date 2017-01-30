//
// Created by eric on 1/29/17.
//
#include <Encoder.h>
#include <Arduino.h>

#ifndef MAIN_ODOMETRY_H
#define MAIN_ODOMETRY_H

class SpeedEncoder
{
    Encoder encoder;

    long lastEncoderPosition = 0;
    long lastEncoderDeltaPosition = 0;
    unsigned long lastEncoderChangeTime = 0;
    unsigned long lastEncoderDeltaTime = 1; // microseconds

    const unsigned long ENCODER_TIMEOUT = 500*long(1000); // microseconds, speed is zero if no reading in this time.
    const long encoderSmoothingTime = 100*long(1000); //microseconds

public:
    // These two numbers control the pins connected to your encoder.
    //   Best Performance: both pins have interrupt capability
    //   Good Performance: only the first pin has interrupt capability
    //   Low Performance:  neither pin has interrupt capability
    SpeedEncoder(int pin1, int pin2){
        encoder = Encoder(pin1, pin2);
    }


    // Returns position in ticks
    float getEncoderVal(){
        return encoder.read();
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
        long val = encoder.read();
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


        long newPosition = encoder.read();
        if (newPosition != oldPosition) {
            oldPosition = newPosition;
            Serial.println(newPosition);
            return true;
        }
        return false;
    }

};



#endif //MAIN_ODOMETRY_H
