## Alpha\_Sensors

---

### Run:

1. Run Roscore:

	```bash
	roscore
	```

1. Run rosserial for arduino:

	```bash
	roslaunch alpha_main rosserial.launch
	```

1. Run other sensors:

	```bash
	roslaunch alpha_sensors sensors.launch
	```

1. Run RVIZ for visualization (perhaps not on the odroid itself)

	```bash
	export ROS_MASTER_URI=http://$(ODROID_IP):11311
	export ROS_IP=$(YOUR_IP)
	rviz -d "$(rospack find alpha_sensors)/launch/view_sensors.rviz" # Visu
	```
