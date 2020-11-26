# Repository for CAS Gazebo/Sensor Fusion project

## How makey work
**Requires:** 
* Ubuntu 18.04
* ROS Melodic
* Lots of time, determination and self-loathing

**Launching:** 
1. Place the repo in catkin workspace (typically `cd ~/catkin_ws/src/`)
2. Make sure you are in catkin workspace (again, `cd ~/catkin_ws/`)
3. Run `source devel/setup.bash`
4. Run `catkin_make`
5. Launch Gazebo simulation using `roslaunch drone_gazebo drone_world.launch`

**Useful stuff:**
* Use `rosrun drone_gazebo controller_HACK.py` to start hacky position PID
    * Publish new setpoints using `rostopic pub /drone/controller/setpoints` (press TAB a few times to autocomplete message template)
* GPS measurements is published to `/drone/sensors/gps`
* IMU measurements with and without gravity are published to `/drone/sensors/imu` and `/drone/sensors/imu_grav`