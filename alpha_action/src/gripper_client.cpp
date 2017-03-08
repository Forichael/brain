#include <ros/ros.h>
#include <alpha_action/GripAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<alpha_action::GripAction> GripperClient;

class Gripper{
	private:
		GripperClient* client;
	public:
		Gripper(){
			client = new GripperClient("alpha_grip", true);
			while(!client->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the alpha_grip action server to come up");
			}
		}
		~Gripper(){
			if(client){
				delete client;
				client = NULL;
			}
		}

		void open(){
			alpha_action::GripActionGoal goal;
			goal.goal.do_grip = false;
			client->sendGoal(goal.goal);
			client->waitForResult();
			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("The Gripper Succeeded To Open!");
			}else{
				ROS_INFO("The Gripper Failed To Open...");
			}
		}

		void close(){
			alpha_action::GripActionGoal goal;
			goal.goal.do_grip = true;
			client->sendGoal(goal.goal);
			client->waitForResult();
			if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("The Gripper Gripped Something!");
			}else{
				ROS_INFO("The Gripper Didn't Grip Anything...");
			}
		}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "alpha_grip_client");
	Gripper gripper;

	gripper.open();
	//gripper.close();

	return 0;
}
