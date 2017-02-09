#include <Encoder.h>
#include <Arduino.h>
#include <sensor_msgs/Range.h>


#define IRPinLeft A1
#define IRPinRight A0

void setupDistanceSensors();
void loopDistanceSensors();
float getRange(int pin_num);
void HandleIR();

sensor_msgs::Range left_range_msg;
sensor_msgs::Range right_range_msg;
ros::Publisher left_pub_range( "/IR/left", &left_range_msg);
ros::Publisher right_pub_range( "/IR/right", &right_range_msg);

void setupDistanceSensors(){
  nh.advertise(left_pub_range);
  nh.advertise(right_pub_range);

  left_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  left_range_msg.header.frame_id = "base_link"; //TODO: update to match urdf
  left_range_msg.field_of_view = 0.01;
  left_range_msg.min_range = 0.03;
  left_range_msg.max_range = 0.8;

  right_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  right_range_msg.header.frame_id = "base_link"; //TODO: update to match urdf
  right_range_msg.field_of_view = 0.01;
  right_range_msg.min_range = 0.03;
  right_range_msg.max_range = 0.8;
}

#define DISTANCE_LOOP_TIME 100 // millis
void loopDistanceSensors(){
    static unsigned long lastUpdateTime = millis();

    if (millis() - lastUpdateTime >= DISTANCE_LOOP_TIME) {
      lastUpdateTime = millis();
      HandleIR();
    }
}

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 * Taken from http://wiki.ros.org/rosserial_arduino/Tutorials/IR%20Ranger
 */
float getRange(int pin_num){
    float sample;
    // Get data
    sample = analogRead(pin_num)/4.0;
    // if the ADC reading is too low, 
    //   then we are really far away from anything
    if(sample < 10)
        return 0;     // max range

    // Magic numbers to get cm
    // TODO: replace with real calibration data
    sample= 1309/(sample-3);
    return (sample - 1)/100; //convert to meters
}

void HandleIR(){
  float IRLeftDistance = getRange(IRPinLeft);
  float IRRightDistance = getRange(IRPinRight);

  //Serial.println(IRLeftDistance);
  //Serial.println(IRRightDistance);


  left_range_msg.range = IRLeftDistance;
  left_range_msg.header.stamp = nh.now();
  left_pub_range.publish(&left_range_msg);


  right_range_msg.range = IRRightDistance;
  right_range_msg.header.stamp = nh.now();
  right_pub_range.publish(&right_range_msg);
}