#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int lh=4, hh=7; // reasonable default values
int ls=4, hs=7; // reasonable default values
int lv=4, hv=7; // reasonable default values

// kernel
cv::Mat ker = cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(3,3), cv::Point(1,1));

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	public:
	ImageConverter()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
	}

	~ImageConverter()
	{

	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Draw an example circle on the video stream
		cv::Mat thresh;
		cv::GaussianBlur(cv_ptr->image, cv_ptr->image,cv::Size(13,13), 0, 0);
		cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV);

		cv::inRange(cv_ptr->image, cv::Scalar(lh,ls,lv), cv::Scalar(hh,hs,hv), thresh);

		cv::erode(thresh, thresh, ker);

		// Update GUI Window
		cv::imshow("img", cv_ptr->image);
		cv::imshow("thresh", thresh);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	cv::namedWindow("ctrl");

	cv::createTrackbar("lh", "ctrl", &lh, 255);
	cv::createTrackbar("hh", "ctrl", &hh, 255);

	cv::createTrackbar("ls", "ctrl", &ls, 255);
	cv::createTrackbar("hs", "ctrl", &hs, 255);

	cv::createTrackbar("lv", "ctrl", &lv, 255);
	cv::createTrackbar("hv", "ctrl", &hv, 255);

	while(ros::ok()){
		ros::spinOnce();
		cv::waitKey(3);
	}

	return 0;
}
