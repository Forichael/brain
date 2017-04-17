#include "alpha_main/pid.h"
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>

#define cap(mn,x,mx) ((x)<(mn)?(mn):(x)>(mx)?(mx):(x))

typedef geometry_msgs::Vector3 v_t;
typedef geometry_msgs::Point p_t;

float d2r(float deg){
	return deg * M_PI / 180;
}

float r2d(float rad){
	return rad * 180 / M_PI;
}

class PathDancerBase{
	public:
		PathDancerBase(){}
		virtual ~PathDancerBase(){}
		virtual void run(std::vector<geometry_msgs::Point>& points)=0; //pure virtual
};

class DeadReckoningDancer : public PathDancerBase{
	private:
		ros::Publisher cmd_pub;
		geometry_msgs::Twist cmd_msg;
		float state[3];
	public:
		DeadReckoningDancer(ros::NodeHandle& nh){
			state[0] = state[1] = state[2] = 0; // not strictly necessary
			cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
		}

		virtual ~DeadReckoningDancer(){}

		virtual void run(std::vector<p_t>& points){
			v_t& l = cmd_msg.linear;
			v_t& a = cmd_msg.angular;
			ros::Rate r(20);
			ros::Time start = ros::Time::now();
			while(start.toSec() == 0){
				r.sleep();
				start = ros::Time::now();
			}

			float dt = 5.0;

			// in C++11
			// for(auto& point : points){
			// // ... content
			// }
			for(std::vector<p_t>::iterator it = points.begin(); it != points.end(); ++it){
				p_t& point = (*it);
				float dx = point.x - state[0];
				float dy = point.y - state[1];
				float theta = atan2(dy,dx) - state[2];
				float delta = sqrt(dx*dx+dy*dy);
				float w = theta/dt;
				float v = delta/dt;

				start = ros::Time::now();
				a.z = w;
				l.x = 0.0;

				while ( (ros::Time::now() - start).toSec() < dt){
					cmd_pub.publish(cmd_msg);
					r.sleep();
				}

				start = ros::Time::now();
				a.z = 0.0;
				l.x = v;

				while ( (ros::Time::now() - start).toSec() < dt){
					cmd_pub.publish(cmd_msg);
					r.sleep();
				}

				state[0] = point.x;
				state[1] = point.y;
				state[2] = atan2(dy,dx);
			}
		}
};

class OdomDancer : public PathDancerBase{
	private:
		ros::Publisher cmd_pub;
		ros::Subscriber odom_sub;
		geometry_msgs::Twist cmd_msg;
		float x0, y0, t0;
		float position[2];
		float orientation;
		bool initialized;
		PID vpid, wpid;

	public:
		OdomDancer(ros::NodeHandle& nh):
			vpid(1.0,0.01,0.0), wpid(1.0,0.01,0.0){
				cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
				odom_sub = nh.subscribe<nav_msgs::Odometry>("odometry/filtered/local", 10, &OdomDancer::odom_cb, this);

				initialized = false;
				position[0] = position[1] = 0; //not necessary
				orientation = 0; //not necessary
			}
		void odom_cb(const nav_msgs::OdometryConstPtr& msg){
			const p_t& p = msg->pose.pose.position;
			const geometry_msgs::Quaternion& q = msg->pose.pose.orientation;

			float x = p.x;
			float y = p.y; 
			float t = atan2(q.z,q.w)*2;

			if(!this->initialized){
				x0 = x; y0 = y; t0 = t;
				this->initialized = true;
			}

			position[0] = x-x0;
			position[1] = y-y0;
			orientation = t-t0;
		}

		virtual ~OdomDancer(){}

		virtual void run(std::vector<p_t>& points){
			ros::AsyncSpinner spinner(1); // thread count
			spinner.start();

			v_t& l = cmd_msg.linear;
			v_t& a = cmd_msg.angular;
			ros::Rate r(20);
			float dt = 1/20.; // dt here is different from dt for DeadReckoningDancer
			float position_tolerance = 0.1;
			float orientation_tolerance = d2r(5);

			while(!initialized){
				//ros::spinOnce(); // need to pump callbacks here
				//alternatively, have ros::AsyncSpinner
				r.sleep();
			}

			for(std::vector<p_t>::iterator it = points.begin(); it != points.end(); ++it){
				p_t& point = (*it);
				wpid.reset();
				while(true){
					float dx = point.x - position[0];
					float dy = point.y - position[1];
					float theta = atan2(dy,dx) - orientation;
					theta = atan2(sin(theta),cos(theta)); // lazy normalization
					//float delta = sqrt(dx*dx+dy*dy);

					if (theta < orientation_tolerance)
						break;
					float w = wpid.compute(theta,dt);
					w = cap(-0.5, w, 0.5);
					a.z = w;
					l.x = 0.0;
					cmd_pub.publish(cmd_msg);
					r.sleep();
				}

				wpid.reset();
				vpid.reset();
				while(true){
					float dx = point.x - position[0];
					float dy = point.y - position[1];
					float theta = atan2(dy,dx) - orientation;
					theta = atan2(sin(theta),cos(theta)); // lazy normalization
					float delta = sqrt(dx*dx+dy*dy);
					if (delta < position_tolerance)
						break;
					float w = wpid.compute(theta,dt);
					float v = vpid.compute(delta,dt);

					w = cap(-0.5,w,0.5);
					v = cap(-0.5,v,0.5);

					a.z = w;
					l.x = v;
					cmd_pub.publish(cmd_msg); // cmd_msg indirectly modified
					r.sleep();
				}
			}
			spinner.stop();
		}
};

class MoveBaseDancer : public PathDancerBase{
	private:
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
		tf::TransformListener tf_listener;
		float x0, y0;
	public:
		MoveBaseDancer():
			client("move_base", true),
			tf_listener(ros::Duration(10)){
			}
		void run(std::vector<p_t>& points){
			tf::StampedTransform t;

			while(true){
				try{
					ros::Time now = ros::Time::now();
					tf_listener.waitForTransform("map","base_link",now,ros::Duration(4.0));
					tf_listener.lookupTransform("map","base_link",now, t);
					break;
				}catch(tf::LookupException& e){
					ROS_INFO("FAILED : %s", e.what());
				}
			}

			x0 = t.getOrigin().getX();
			y0 = t.getOrigin().getY();
			client.waitForServer();

			move_base_msgs::MoveBaseGoal goal;
			for(std::vector<p_t>::iterator it = points.begin(); it != points.end(); ++it){
				p_t& point = (*it);
				make_goal(goal,point);
				client.sendGoalAndWait(goal);
			}

		}

		void make_goal(move_base_msgs::MoveBaseGoal& goal, p_t& p){
			float x = p.x;
			float y = p.y;
			x += x0;
			y += y0;
			float theta = atan2(y,x);

			geometry_msgs::Quaternion orientation;
			orientation.z = sin(theta/2);
			orientation.w = cos(theta/2);

			geometry_msgs::PoseStamped target_pose;
			target_pose.header.stamp = ros::Time::now();
			target_pose.header.frame_id = "map";

			p_t position;
			position.x = x;
			position.y = y;
			position.z = 0.0;

			target_pose.pose.position = position;
			target_pose.pose.orientation = orientation;
			goal.target_pose = target_pose;
		}
};

std::vector<p_t> make_path(){
	std::vector<p_t> res;
	float r = 2.0, theta = 0.0;
	for(int i=0; i<6; ++i){
		theta = i * 144 * M_PI / 180;
		float x = r * cos(theta);
		float y = r * sin(theta);
		geometry_msgs::Point p;
		p.x = x; p.y = y; p.z = 0.0;
		res.push_back(p);
		//res.emplace_back(x,y,0);
	}
	geometry_msgs::Point p;
	p.x = 0.0; p.y = 0.0; p.z = 0.0;
	res.push_back(p);
	return res;
}

int main(int argc, char* argv[]){
	ros::init(argc,argv,"alpha_dance");
	ros::NodeHandle nh;
	//PathDancerBase* dancer = new DeadReckoningDancer(nh);
	//PathDancerBase* dancer = new OdomDancer(nh);
	PathDancerBase* dancer = new MoveBaseDancer();
	std::vector<p_t> path = make_path();
	dancer->run(path);
	delete dancer;
}
