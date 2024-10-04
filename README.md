# AMR Simulation
This is for AMR.

## 🛠️Prerequisites(UI)
### 1) PyQt
Test environment is PyQt5.

### 2) OpenCV
Test environment is OpenCV 4.4  
It will work well in the Opencv 4.X

### 3) ROS
Test environment is ROS Noetic.  
It will work well in the ROS1(not ROS2).

### 4) Octomap
```bash
$ sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-msgs ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server
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
- (optional) roboy keyboard control
```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/ddmr_velocity_controller/cmd_vel
```