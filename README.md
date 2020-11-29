# Repository for CAS Gazebo/Sensor Fusion project

## Gazebo
### Requirements
* Ubuntu 18.04
* ROS Melodic

### Launching Gazebo 
1. Extract the repo into catkin workspace (typically `cd ~/catkin_ws/src/`)
2. Make sure you are in catkin workspace (again, `cd ~/catkin_ws/`)
3. Run `source devel/setup.bash`
4. Run `catkin_make`
5. Launch Gazebo simulation using `roslaunch drone_gazebo drone_world.launch`

### Useful information
* Use `rosrun drone_gazebo controller.py` to start position PID controller
    * Publish new setpoints using `rostopic pub /drone/controller/setpoints` (press TAB a few times to autocomplete message template)
* GPS measurements is published to `/drone/sensors/gps`
* IMU measurements with and without gravity are published to `/drone/sensors/imu` and `/drone/sensors/imu_grav`
* Noise can be simulated by changing values of `gaussianNoise` parameters in `drone.gazebo`
