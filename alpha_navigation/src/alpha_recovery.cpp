#ifndef __ALPHA_RECOVERY_H__
#define __ALPHA_RECOVERY_H__
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>

namespace alpha_recovery{
	class AlphaRecovery : public nav_core::RecoveryBehavior{

		private:
			bool initialized_;
			bool enabled_;
			int timeout_;
			int num_attempts_;

			std::string name_;
			tf::TransformListener* tf_;
			costmap_2d::Costmap2DROS* global_costmap_;
			costmap_2d::Costmap2DROS* local_costmap_;
			base_local_planner::CostmapModel* local_costmap_model_;	

			geometry_msgs::PoseStamped last_goal_;


		public:

			AlphaRecovery();
			virtual ~AlphaRecovery();

			virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
			virtual void runBehavior();

	};
};

//PLUGINLIB_DECLARE_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
#endif


namespace alpha_recovery{
	AlphaRecovery::AlphaRecovery(): initialized_(false){

	}
	AlphaRecovery::~AlphaRecovery(){

	}
	void AlphaRecovery::initialize(std::string name, tf::TransformListener* tf, \
			costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
		if(initialized_)
			return;
		initialized_ = true;
		name_ = name;
		tf_ = tf;
		global_costmap_ = global_costmap;
		local_costmap_ = local_costmap;
		local_costmap_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

	}

	void AlphaRecovery::runBehavior(){
		if (!initialized_)
		{
			ROS_ERROR(" [Alpha Recovery] Not Initialized : Not running");
			return;
		}

		if (!enabled_)
		{
			ROS_INFO(" [Alpha Recovery] Is Disabled : Not running", name_.c_str());
			return;
		}

	}
};
PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)

