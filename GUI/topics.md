The GUI subscribes to several topics do its job.

# Status indicators
Messages on the following topics turn the status indicators green:

- USB hub: `/imu/data` (type sensor_msgs/Imu)
- Arduino communications: `/rwheel` (type std_msgs/Int16)
- LIDAR data: `/scan` (type sensor_msgs/LaserScan)
- Localization: `/odometry/filtered` (type nav_msgs/Odometry)
- "Got that Can" : '/we_got_the_can' (type std_msgs/Bool) 
- Mission complete : '/mission_complete' (type std_msgs/Bool) 
