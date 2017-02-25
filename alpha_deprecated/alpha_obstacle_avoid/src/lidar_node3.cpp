//
// Created by cedrickim on 12/4/16.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include <math.h>

#define PI 3.1415926535

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Publisher *velocity_data_ptr;
ros::Publisher *bool_data_ptr;

float robotLength = 0.5; // meters
float robotLengthTolerance = 0.1; //meters
float totalDetectionWidth = robotLength + robotLengthTolerance; // meters
int turnScaleFactor = 2;
float angleThreshold = 120 * (PI / 180);
float rMin = 2.5;
int smallestDetectionRange = .5;
//Output of 11 values, robot vision split in to two, left and right. Object detected on left side = turn right, object on
//right = turn left. position in 11 val array moves based on how close the object is in the left/right detection. weight is determined by
//outside of the range, of x positions, value gets a sum based off of other values.

struct XandRValues{
    float x;
    float r;
    float theta;
};
XandRValues data;

float scaleWeightValues(float input) {
    //returns a value, where higher radius value = lower  return value (between 0 and 5)
    float output = 5 - (pow(input , 2) * (5 + smallestDetectionRange) / pow(rMin, 2) - smallestDetectionRange);
    if (output < 0){
        return 0;
    }
    if (output > 5){
        return 5;
    }
    return output;
}

bool whichisbigger(const XandRValues& a, const XandRValues& b){
    return a.x < b.x;
}

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    float angleMin = msg->angle_min;
    float angleIncrement = msg->angle_increment;
    int zeroIndex = -angleMin / angleIncrement;
    std::vector<float> ranges = msg->ranges;
    int startIndex = zeroIndex - (angleThreshold / 2) / angleIncrement; //limits to angleThreshold
    int endIndex = zeroIndex + (angleThreshold / 2) / angleIncrement;   //limits to angleThreshold
    std::vector <XandRValues> leftValues;
    std::vector <XandRValues> rightValues;

    for (int i = endIndex; i > startIndex; i--) {
        if (ranges[i] > .03) {
            float currentAngle = (zeroIndex - i) * angleIncrement;
            float x = ranges[i] * sin(currentAngle);
            if (x > 0) {
                data.x = x;
                data.r = ranges[i];
                data.theta = currentAngle*180/PI;
                rightValues.push_back(data);
            }else{
                data.x = -x;
                data.r = ranges[i];
                data.theta = -currentAngle*180/PI;
                leftValues.push_back(data);
            }
        }
    }

    std::sort(rightValues.begin(), rightValues.end(), whichisbigger);
    std::sort(leftValues.begin(), leftValues.end(), whichisbigger);

    std::vector<float> obsTurnArray(11);


    float rMinRobot = rMin;
    float weight = 0;
    float stepAngle = 5; //width of each step in meters, to be added onto the weight.
    int stepIndex = 0;
    int position = 0;
    float trueRMin = rMin;
    float rMinRight = rMin;
    float rMinLeft = rMin;
    bool detected = false;
    //Runs through values, if within half robot length, we grab that r value. The position depends on inverse squared.
    //In addition, runs through the rest of the values, as a step. We add these values onto the weight of the thing.

    //--------------------//
    //RIGHT SIDE OF ROBOT//
    //-------------------//
    for (int i = 0; i < rightValues.size(); i++) {
        if (rightValues[i].x < totalDetectionWidth/2){
            if(rightValues[i].r < rMinRobot){
                detected = true;
                rMinRobot = rightValues[i].r;
                weight =  scaleWeightValues(rMinRobot);
                trueRMin = rMinRobot;
            }
        }else{
            //std::cout << rightValues[i].theta << std::endl;
            if(stepIndex*stepAngle - rightValues[i].theta > stepAngle){
                if(rightValues[i].r < rMinRight) {
                    rMinRight = rightValues[i].r;
                }
            }else{
                weight += scaleWeightValues(rMinRight);
                std::cout << scaleWeightValues(rMinRight) << std::endl;
                stepIndex ++;
                rMinRight = rMin;
            }
        }
    }

    //std::cout << rMinRobot << std::endl;
    position = int(scaleWeightValues(rMinRobot));
    //std::cout << position << std::endl;
    obsTurnArray[5 - position] = (weight/10);
    stepIndex = 0;
    weight = 0;
    rMinRobot = rMin;
    //repeat for left side
    for (int i = 0; i < leftValues.size(); i++) {
        if (leftValues[i].x < totalDetectionWidth/2){
            detected = true;
            if(leftValues[i].r < rMinRobot){
                rMinRobot = leftValues[i].r;
                weight = scaleWeightValues(rMinRobot);
                trueRMin = rMinRobot;
            }
        }else{

            if(stepIndex*stepAngle - leftValues[i].theta > stepAngle){
                if(leftValues[i].r < rMinLeft) {
                    rMinLeft = leftValues[i].r;
                }
            }else{
                weight += scaleWeightValues(rMinLeft);
                stepIndex ++;
                rMinLeft = rMin;
            }
        }
    }
    position = int(scaleWeightValues(rMinRobot));

    //std::cout << position << std::endl;
    obsTurnArray[5 + position] = (weight/10);
    obsTurnArray[5] = .01;
    std::vector<float> obsVelArray(11);

    int velocityIndex;

    velocityIndex = int((trueRMin/rMin)*11) + 5;
    std::cout << trueRMin << std::endl;
    if (velocityIndex > 10) {
        velocityIndex = 10;
    }
    else if (velocityIndex < 0){
        velocityIndex = 0;
    }

    obsVelArray[velocityIndex] = 1;



    std::vector<float> finalObsArray;
    finalObsArray.insert(finalObsArray.end(), obsVelArray.begin(), obsVelArray.end() );
    finalObsArray.insert(finalObsArray.end(), obsTurnArray.begin(), obsTurnArray.end() );

    std_msgs::Float32MultiArray msg_out;
    msg_out.data = finalObsArray;

    for (int i = 11; i < 22; i++) {
        std::cout << finalObsArray[i];
        std::cout << ", ";
        if (i == 21){
            std::cout << "" << std::endl;
        }
    }
    std_msgs::Bool detectedBool;
    detectedBool.data = detected;
    bool_data_ptr ->publish(detectedBool);
    velocity_data_ptr->publish(msg_out);

}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "listener");
    std::cout << "Initializing" << std::endl;

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
    ros::Publisher velocity_data = n.advertise<std_msgs::Float32MultiArray>("obst/cmd_vel", 1000);
    ros::Publisher bool_data = n.advertise<std_msgs::Bool>("obst/detected", 1000);
    std::cout << "Streaming Data" << std::endl;
    velocity_data_ptr = &velocity_data;
    bool_data_ptr = &bool_data;

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}