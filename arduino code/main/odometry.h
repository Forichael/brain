#include "encoders.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>


// Setup publishers and subscribers
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

#ifndef HAS_LEFT_ENCODER
double angleVel = 0;
void imuCallback(const sensor_msgs::Imu& msg){
    angleVel = -msg.angular_velocity.z; // Negative because the sensor is upside-down
}
ros::Subscriber<sensor_msgs::Imu> imuSub("imu", &imuCallback);
#endif

// Setup encoder data
SpeedEncoder rightEncoder(20, 21);

#ifdef HAS_LEFT_ENCODER
SpeedEncoder leftEncoder(18, 19);
#endif

const double TICKS_PER_METER = 12 / 0.399; // (ticks / circumference)
const double WHEELBASE = .7; // in meters

unsigned long current_time, last_time;
double x=0, y=0, th=0;


// Helper functions
geometry_msgs::Quaternion quaternionFromYaw(double th){
    // Taken from http://answers.ros.org/question/9772/quaternions-orientation-representation/?answer=14285#post-id-14285
    geometry_msgs::Quaternion quat;
    quat.x = quat.y = 0;
    quat.z = sin(th/2);
    quat.w = cos(th/2);

    return quat;
}

void updateOdometry(){
    
    current_time = micros();

    double right_wheel_speed = rightEncoder.getEncoderSpeed() / TICKS_PER_METER;

    #ifdef HAS_LEFT_ENCODER
    double left_wheel_speed = leftEncoder.getEncoderSpeed() / TICKS_PER_METER;
    #else
    double left_wheel_speed = right_wheel_speed - angleVel * WHEELBASE;
    #endif

    double vx = (right_wheel_speed + left_wheel_speed) / 2.0;
    double vy = 0;
    double vth = (right_wheel_speed - left_wheel_speed) / 2.0;

    
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time) / double(1e6);
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = quaternionFromYaw(th);

    //Publish the odometry message over ROS
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "odom";

    //set the position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(&odom_msg);

    last_time = current_time;
}


// Main functions
void setupOdometry(){
    current_time = last_time = micros();
}

const int ODOM_LOOP_TIME = 1000; // ms

void loopOdometry(){
    rightEncoder.loopEncoder();
    #ifdef HAS_LEFT_ENCODER
    leftEncoder.loopEncoder();
    #endif

    static unsigned long lastOdomTime = millis();
    
    if (millis() - lastOdomTime >= ODOM_LOOP_TIME) {
        lastOdomTime = millis();
        updateOdometry();
    }
}