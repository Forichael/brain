#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"

double then;

double pos_x, pos_y;
double vel_x, vel_y; // implicitly assumed zero

nav_msgs::Odometry odom_msg;
geometry_msgs::Vector3 accel_msg;

void magCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	//ROS_INFO("Imu Magnetic Orientation x: [%f], y: [%f], z: [%f]", msg->vector.x,msg->vector.y,msg->vector.z);
	return;
}

void data_RawCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	const ros::Time& t = msg->header.stamp;

	double now = t.toSec();

	double dt = now - then;

	// convert to global ...
	tf::Quaternion quat;
	tf::Vector3 vec;

	tf::quaternionMsgToTF(msg->orientation, quat);
	tf::vector3MsgToTF(msg->linear_acceleration, vec);
	tf::vector3TFToMsg(tf::quatRotate(quat, vec),accel_msg);
	
	// now integrate
	vel_x += accel_msg.x * dt;
	vel_y += accel_msg.y * dt;

	vel_x *= 0.999; // anneal
	vel_y *= 0.999;

	pos_x += vel_x * dt;
	pos_y += vel_y * dt;

	// fill in header
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";

	// fill in pose
	odom_msg.pose.pose.position.x = pos_x;
	odom_msg.pose.pose.position.y = pos_y;
	odom_msg.pose.pose.position.z = 0.0; // ignore z component
	odom_msg.pose.pose.orientation = msg->orientation;

	// fill in twist
	odom_msg.twist.twist.linear.x = vel_x;
	odom_msg.twist.twist.linear.y = vel_y;
	odom_msg.twist.twist.linear.z = 0.0; // ignore z component
	odom_msg.twist.twist.angular = msg->angular_velocity; // directly use angular velocity

	// empty covariance, needs to be dealt with.
	// for pose/twist

	then = now;
	return;
}

void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	ROS_INFO("GPS FIX STATUS: [%d], [%d]", msg->status.status, msg->status.service);
	ROS_INFO("Lat: [%f] Lon: [%f]", msg->latitude, msg->longitude);
	return;
}

void gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	ROS_INFO("Linear");

	return;
}

int main(int argc, char* argv[])
{
  //initialize the node. 
  ros::init(argc, argv, "midbrain");
  //node handle.
  ros::NodeHandle n;
  //subscribes to a node.
  then = ros::Time::now().toSec();
  odom_msg.header.frame_id = "odom";

  ros::Subscriber imuSub = n.subscribe("/imu/mag", 1000, magCallback);
  ros::Subscriber magSub = n.subscribe("/imu/data", 1000, data_RawCallback);
  ros::Subscriber gpsFixSub = n.subscribe("fix", 1000, gpsFixCallback);
  ros::Subscriber gpsVelSub = n.subscribe("vel", 1000, gpsVelCallback);
  //wait for callbacks

  ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("odom", 1000, true);
  ros::Publisher orPub = n.advertise<geometry_msgs::Vector3>("lin_accel", 1000, true);

  while(ros::ok()){
  	odomPub.publish(odom_msg);
	orPub.publish(accel_msg);
  	ros::spinOnce();
  }

	return 0;
}
