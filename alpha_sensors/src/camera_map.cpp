#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <boost/shared_ptr.hpp>

int main(int argc, char* argv[]){
	ros::init(argc, argv, "camera_map");
	tf::TransformListener tf(ros::Duration(10));
	boost::shared_ptr<costmap_2d::Costmap2DROS> costmap = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("", tf));
	costmap->resetLayers();
	ros::spin();
}
