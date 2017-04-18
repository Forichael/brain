/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, JSK (The University of Tokyo).
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"
#include "pluginlib/class_list_macros.h"

namespace alpha_sensors
{

	class CameraFilter : public filters::FilterBase<sensor_msgs::LaserScan>
	{
		public:
			double fov_;
			double lower_threshold_ ;
			double upper_threshold_ ;

			bool configure()
			{
				// default values
				lower_threshold_ = 0.0;
				upper_threshold_ = 100000.0;
				fov_ = 1.0; // ~60 deg.

				// load from given parameters
				bool success =
					getParam("lower_threshold", lower_threshold_) &&
					getParam("upper_threshold", upper_threshold_) &&
					getParam("fov", fov_);
				return success;
			}

			virtual ~CameraFilter()
			{

			}

			bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
			{
				filtered_scan.ranges.resize(input_scan.ranges.size());
				filtered_scan.intensities.resize(input_scan.intensities.size());

				double start_angle = input_scan.angle_min;
				double current_angle = input_scan.angle_min;
				ros::Time start_time = input_scan.header.stamp;

				double lower_angle = (-fov_/2);
				double upper_angle = (fov_/2);
				unsigned int count = 0;

				const float& inc = input_scan.angle_increment;

				for (unsigned int i=0; i < input_scan.ranges.size();++i) // Need to check ever reading in the current scan
				{
					if(current_angle > upper_angle){
						// now out of range
						current_angle -= inc;
						break;
					}
					if(start_angle < lower_angle){
						// not in range yet
						start_angle += inc;
						current_angle += inc;
						start_time += ros::Duration(input_scan.time_increment);
					}else{
						// within range !!
						float& r = filtered_scan.ranges[count];
						r = input_scan.ranges[i];

						if (r <= lower_threshold_ )
						{
							// if lower than thresh, then invalid
							r = std::numeric_limits<float>::quiet_NaN();
						}
						else if(r >= upper_threshold_){
							// if higher than thresh, then "valid" -- just cap it
							r = std::numeric_limits<float>::infinity(); //upper_threshold_ - 1e-3;
						}

						current_angle += inc;
						++count;
					}
				}

				// fill in the parameters
				filtered_scan.header.frame_id = input_scan.header.frame_id;
				filtered_scan.header.stamp = start_time;

				filtered_scan.angle_min = start_angle;
				filtered_scan.angle_max = current_angle;
				filtered_scan.angle_increment = input_scan.angle_increment;
				filtered_scan.time_increment = input_scan.time_increment;
				filtered_scan.scan_time = input_scan.scan_time;

				filtered_scan.range_min = lower_threshold_;
				filtered_scan.range_max = upper_threshold_;

				filtered_scan.ranges.resize(count);
				filtered_scan.intensities.resize(count);

				return true;
			}
	} ;

}

PLUGINLIB_REGISTER_CLASS(alpha_sensors/CameraFilter, alpha_sensors::CameraFilter, filters::FilterBase<sensor_msgs::LaserScan>)
