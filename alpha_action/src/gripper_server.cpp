#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <alpha_action/GripAction.h>
#include <std_msgs/Bool.h>

class GripAction{
	protected:
		ros::NodeHandle nh_;

		std_msgs::Bool grip_msg;
		ros::Publisher g_pub;
		ros::Subscriber g_sub;

		actionlib::SimpleActionServer<alpha_action::GripAction> as_;
		std::string action_name_;
		alpha_action::GripFeedback feedback_;
		alpha_action::GripResult result_;

		bool sw_state; // current limit switch state

	public:
		GripAction(std::string name):
			as_(nh_, name, boost::bind(&GripAction::executeCB, this, _1), false),
			action_name_(name){
				g_pub = nh_.advertise<std_msgs::Bool>("grip", 10);
				g_sub = nh_.subscribe<std_msgs::Bool>("lim_sw", 10, &GripAction::sw_cb, this);
				as_.start();
			}
		~GripAction(){
		}

		void sw_cb(const std_msgs::BoolConstPtr& msg){
			sw_state = msg->data;
		}

		void executeCB(const alpha_action::GripGoalConstPtr& goal){

			grip_msg.data = goal->do_grip;
			g_pub.publish(grip_msg);

			feedback_.done = false;

			ros::Time begin = ros::Time::now();

			ros::Rate hz(20);

			float timeout = 1.0; // wait 1 sec
			// current servo spec : 60 deg / 0.2 sec
			// 1 sec should be quite enough

			while((ros::Time::now() - begin).toSec() < timeout){
				if(sw_state){
					result_.gripped = true;
					as_.setSucceeded(result_);
					return;
				}	
				if(as_.isPreemptRequested() || !ros::ok()){
					ROS_INFO("%s: Preempted", action_name_.c_str());
					// set the action state to preempted
					as_.setPreempted();
					break;
				}
				as_.publishFeedback(feedback_);
				hz.sleep();
			}

			// if no response from limit switch, then obviously something went wrong
			result_.gripped = false;
			as_.setAborted(result_);

		}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "alpha_grip");
	GripAction grip("alpha_grip");
	ros::spin();
	return 0;
}
