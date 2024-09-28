# AMR UI
This is UI for AMR.

## 🛠️Prerequisites
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

## 🛠️Build
```bash
$ cd (YOUR_WORKSPACE)/src
$ git clone https://github.com/SanghyunPark01/amr_software.git
$ git checkout amr_ui
$ cd ..
$ catkin_make
```  

## 🛠️Run
```bash
$ roslaunch amr_ui test.launch
```