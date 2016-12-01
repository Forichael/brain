roscore
export ROS_NAMESPACE=my_camera
rosrun uvc_camera uvc_camera_node _device:="/dev/video1"
rosrun camera_node record_frames.py 


To start Recording:
	rostopic pub /start std_msgs/Bool True 
		(or equivalant)

To end Recording:
	rostopic pub /end std_msgs/Bool True