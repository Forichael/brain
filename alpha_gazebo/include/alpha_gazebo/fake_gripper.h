#ifndef __ALPHA_FAKE_GRIPPER_H__
#define __ALPHA_FAKE_GRIPPER_H__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include <stdio.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo{
	class AlphaFakeGripper: public ModelPlugin{
		protected:
			ros::NodeHandle nh_;
			ros::Publisher l_pub; // "lim_sw"
			ros::Subscriber g_sub; // grip/release upon "grip"
		public:
			AlphaFakeGripper();
			virtual ~AlphaFakeGripper();
			void grip_cb(const std_msgs::BoolConstPtr& msg);
			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
			void OnUpdate(const common::UpdateInfo& _info);
		private:
			physics::ModelPtr model;
			physics::LinkPtr base_link;
			physics::JointPtr gripper_joint;
			event::ConnectionPtr update_connection;
			std::string base_link_name;
			bool attached;
			std::string object_name;
	};

	GZ_REGISTER_MODEL_PLUGIN(AlphaFakeGripper)
}
#endif
