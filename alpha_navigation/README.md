## Alpha\_Navigation

Navigation Stack for Alphabot using Move\_Base

### To Run:


If you're running SLAM (i.e. slam\_gmapping, etc.) to produce map and localization from map to odom :

```bash
roslaunch alpha_navigation move_base.launch slam:=true
```

Otherwise, if you're running map-server for a static map and localization from amcl:

```bash
roslaunch alpha_navigation move_base.launch #slam:=false
```

Slam is disabled by default.

### Error Msg:

```bash
terminate called after throwing an instance of 'pluginlib::LibraryLoadException'
  what():  Could not find library corresponding to plugin costmap_2d::ObstacleLayer. Make sure the plugin description XML file has the correct name of the library and that the library actually exists.
[31m[move_base-3] process has died [pid 27843, exit code -6, cmd /opt/ros/indigo/lib/move_base/move_base odom:=odometry/filtered/local __name:=move_base __log:=/home/odroid/.ros/log/15e1f400-ff40-11e6-89cf-001e063054ca/move_base-3.log].
log file: /home/odroid/.ros/log/15e1f400-ff40-11e6-89cf-001e063054ca/move_base-3*.log[0m
```
