//
// Created by cedrickim on 12/4/16.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <math.h>

#define PI 3.1415926535

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Publisher *velocity_data_ptr;

float robotLength = 0.5; // meters
float robotLengthTolerance = 0.1; //meters
float totalDetectionWidth = robotLength + robotLengthTolerance; // meters
float optimalRange = 1.75;
float detectionAngle = 85 * (PI/180.0);
float detectionAngleSliceOffset = 5 * (PI/180);
float detectionRange;
float maxDetectionRange;
float angleThreshold = 120 * (PI/180.0);
float gapDetectionAngle = 80 * (PI/180.0);
float gapDetectionSliceOffset = 5 * (PI/180);
const float gapThreshold = 1.25;
int velocityIndex;
int turnIndex;
float dangerDistance = 0.5;
float previousDistance = 1.0;
bool goStraight = false;

//Output of 11 values, robot vision split in to two, left and right. Object detected on left side = turn right, object on
//right = turn left. position in 11 val array moves based on how close the object is in the left/right detection. weight is determined by
//outside of the range, of x positions, value gets a sum based off of other values.

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {


    float angleMin = msg->angle_min;
    float angleIncrement = msg->angle_increment;
    int zeroIndex = -angleMin / angleIncrement;
    int startIndex = zeroIndex - (angleThreshold / 2) / angleIncrement; //limits to angleThreshold
    int endIndex = zeroIndex + (angleThreshold / 2) / angleIncrement;   //limits to angleThreshold
    int detectionDegreeIndex = zeroIndex + (detectionAngle / angleIncrement); // monitoring left side of the robot
    int gapDetectionIndex = zeroIndex + (gapDetectionAngle / angleIncrement); // watching for gaps in the track
    std::vector<float> ranges = msg->ranges;

    float closestRange = 999;
    int closestAngle;
    for(int i = endIndex; i > startIndex; i--){
        if (ranges[i] < closestRange && ranges[i] > 0.03){
            closestRange = ranges[i];
        }
    }

    velocityIndex = int((closestRange/dangerDistance)*11);
    if (velocityIndex > 10) {
        velocityIndex = 10;
    }
    else if (velocityIndex < 5){
        velocityIndex = 5;
    }

    //std::cout << ranges[detectionDegreeIndex] << std::endl;
    std::cout << optimalRange << std::endl;
    //std::cout << closestAngle << std::endl;
    detectionRange = 999;
    maxDetectionRange = ranges[detectionDegreeIndex];
    for(int i = detectionDegreeIndex + detectionAngleSliceOffset; i > detectionDegreeIndex - detectionAngleSliceOffset; i--){
        if (ranges[i] < detectionRange){
            detectionRange = ranges[i];
        }
    }

    turnIndex = int((detectionRange/optimalRange)*5);
    std::vector<float> obsTurnArray(11);
    if (turnIndex > 7) {
        turnIndex = 7;
    }
    else if (turnIndex < 3){
        turnIndex = 3;
    }

    int counter = 0;
    float gapAvg = 0.0;
    for(int i = gapDetectionIndex + gapDetectionSliceOffset; i > gapDetectionIndex - gapDetectionSliceOffset; i--){
        counter++;
        gapAvg += ranges[i];
    }

    gapAvg /= (float)counter;
    //std::cout << gapDetectionIndex << std::endl;
    if((gapAvg - previousDistance) > gapThreshold && !goStraight){
        goStraight = true;
		std::cout<<"GAP!"<<std::endl;
    }
    else if((previousDistance - gapAvg) > gapThreshold && goStraight){
        goStraight = false;
    }

    std::vector<float> obsVelArray(11);
    obsVelArray[velocityIndex] = 1.0;

    previousDistance = gapAvg;

    if(goStraight){
        obsTurnArray[turnIndex] = .001;
    }
    else{
        obsTurnArray[turnIndex] = 0.5;
    }

    std::reverse(obsTurnArray.begin(), obsTurnArray.end());

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
    ros::Publisher velocity_data = n.advertise<std_msgs::Float32MultiArray>("wall/cmd_vel", 1000);
    std::cout << "Streaming Data" << std::endl;
    velocity_data_ptr = &velocity_data;

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
