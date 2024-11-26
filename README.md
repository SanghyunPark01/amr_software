# AMR Simulation
This is for AMR.

## Description
### UI
### GAZEBO
- Input Topic: `/ddmr_velocity_controller/cmd_vel`
- Output Topic: 
    - LiDAR: `/livox/lidar
    - IMU: /imu
    - Camera: same realsense ros

## 🛠️Prerequisites(UI)
### 1) PyQt
Test environment is PyQt5.

### 2) OpenCV
Test environment is OpenCV 4.4  
It will work well in the Opencv 4.X

### 3) ROS
Test environment is ROS Noetic.  
It will work well in the ROS1(not ROS2).  

### 4) Open3D
```bash
$ pip3 install open3d
```

### 5) Octomap
```bash
$ sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-msgs ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server
```  

### 6) etc
```
$ pip3 install pyqtgraph
```

## 🛠️Prerequisites(GAZEBO)
```bash
sudo apt-get install libignition-math4-dev
```

## 🛠️Build
```bash
$ cd (YOUR_WORKSPACE)/src
$ git clone https://github.com/SanghyunPark01/amr_software.git
$ git checkout simulation
$ cd ..
$ catkin_make
```  

- copy gazebo model to your `~/.gazebo/models/`
```bash
$ cp -r src/gazebo_simulation/livox_laser_simulation/models/barn ~/.gazebo/models/
```

## 🛠️Run(UI)
```bash
$ roslaunch amr_ui test.launch
```
## 🛠️Run(GAZEBO)
```bash
$ roslaunch livox_laser_simulation barn_world.launch
```
- (optional) robot keyboard control
```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/ddmr_velocity_controller/cmd_vel
```


## Navigation part
- sensor topic from gazebo
    - /livox/lidar
    - /imu

- Input for Navigation
    - global_map: tf from map to base_link
    - local_map: tf from camera_init to base_link
    - goal: move_base_simple/goal(geometry_msgs/PoseStamped)
    - add waypoint : /add_pose(geometry_msgs/PoseStamped) or 2D Pose Estimate in RVIZ 
    - add waypoints : /waypoints(geometry_msgs/PoseArray)
    - execute: /operation_mode (std_msgs/Int32), stop: 0, start: 1,  repeat: 2, pause: 3

- notes
    - what I've changed or what you need to change are with "@@HERE@@" tag
    - movement in gazebo is wierd now, need to modify
    - fast_lio mapping and localization are all modified for gravity alignment. Ask more detail to sanghyun
    - navigation part is not finished yet. 
    - after mapping you have to change path for map in navigation package
    - if you pause navigator and restart it will follow the last goal(goal before pause)
    - if you stop navigator it will initialize waypoints
        

## 🛠️Run(Mapping)

```bash
$ roslaunch fast_lio mapping_mid360.launch
```

## 🛠️Run(Localization)

```bash
$ roslaunch fast_lio_localization localization_mid360.launch
```


## 🛠️Run(Navigation)

```bash
$ roslaunch nav navigation.launch 2> >(grep -v TF_REPEATED_DATA buffer_core) 
```
