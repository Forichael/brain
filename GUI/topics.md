The GUI subscribes to several topics do its job.

# Status indicators
Messages on the following topics turn the status indicators green:
- ROS master: `/clock` (type rosgraph_msgs/Clock)
- Arduino communications: `/rwheel` (type std_msgs/Int16)
- LIDAR data: `/scan` (type sensor_msgs/LaserScan)
- Can search algorithm: `/can_point` (type geometry_msgs/PointStamped)
