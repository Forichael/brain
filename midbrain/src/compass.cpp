#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"

struct Point{
	double x;
	double y;
	double z;
	Point& operator+=(const Point& rhs){
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}
	Point& operator-=(const Point& rhs){
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}
	Point& operator%=(const Point& rhs){
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
		return *this;
	}
};

template<typename T>
T max(const T& x, const T& y){
	return x>y?x:y;
}

template<typename T>
T min(const T& x, const T& y){
	return x<y?x:y;
}

template<>
Point max(const Point& lhs, const Point& rhs){
	Point res = {max(lhs.x,rhs.x), max(lhs.y,rhs.y), max(lhs.z,rhs.z)};
	return res;
}

template<>
Point min(const Point& lhs, const Point& rhs){
	Point res = {min(lhs.x,rhs.x), min(lhs.y,rhs.y), min(lhs.z,rhs.z)};
	return res;
}

const Point imu_offset = {0.65f, -0.465f, 0.33f};
const Point imu_scale = {2.0 / 0.41f, 2.0 / 0.51f,  2.0f}; // TODO : fix z scale
const float heading_offset = 0.99;

Point imu_max = {.86f, -.21f, .19f};
Point imu_min = {.45f, -.72f, .01f};

ros::Publisher compass_pub;
std_msgs::Float32 compass_msg;

geometry_msgs::Vector3 convert(const Point& p){
	geometry_msgs::Vector3 res;
	res.x = p.x;
	res.y = p.y;
	res.z = p.z;
	return res;
}

void magCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{

	Point norm_mag = {msg->vector.x, msg->vector.y, msg->vector.z};

	norm_mag -= imu_offset;
	norm_mag %= imu_scale;

	float heading = atan2(norm_mag.y, norm_mag.x);
	heading -= heading_offset;
	float heading_norm = atan2(sin(heading), cos(heading)); // slow but convenient

	compass_msg.data = heading_norm;
	compass_pub.publish(compass_msg);

	return;
}


int main(int argc, char* argv[]){
	ros::init(argc,argv,"heading");
	ros::NodeHandle n;
	ros::Subscriber mag_sub = n.subscribe("/imu/mag", 1000, magCallback);
	compass_pub = n.advertise<std_msgs::Float32>("/imu/heading", 1000, false);
	ros::spin();
}
