#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <alpha_action/GripAction.h>
#include <std_msgs/Bool.h>

class GripAction{
	protected:
		ros::NodeHandle nh_;

		std_msgs::Bool grip_msg;
		ros::Publisher g_pub;
		//ros::Subscriber g_sub;

		actionlib::SimpleActionServer<alpha_action::GripAction> as_;
		std::string action_name_;
		alpha_action::GripFeedback feedback_;
		alpha_action::GripResult result_;

	public:
		GripAction(std::string name):
			as_(nh_, name, boost::bind(&GripAction::executeCB, this, _1), false),
			action_name_(name){
				g_pub = nh_.advertise<std_msgs::Bool>("grip", 100);
				as_.start();
			}
		~GripAction(){

		}
		void executeCB(const alpha_action::GripGoalConstPtr& goal){

			grip_msg.data = goal->do_grip;
			g_pub.publish(grip_msg);

			feedback_.done = false;

			while(!feedback_.done){
				feedback_.done = true;
				// TODO : set this code to something more reasonable
				// like the limit switch
				
				if(as_.isPreemptRequested() || !ros::ok()){
					ROS_INFO("%s: Preempted", action_name_.c_str());
					// set the action state to preempted
					as_.setPreempted();
					break;
				}
				as_.publishFeedback(feedback_);
			}

			bool gripped = true; // whether or not it gripped, or just closed the claw
			// this would depend on where the limit switch was triggered
			// TODO : figure this out in a more reasonable way

			if(gripped){
				result_.gripped = true;
				as_.setSucceeded(result_);
			}else{
				result_.gripped = false;
				as_.setAborted(result_);
			}

		}
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "alpha_grip");
	GripAction grip("alpha_grip");
	ros::spin();
	return 0;
}
