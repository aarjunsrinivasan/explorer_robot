# Explorer_robot
[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/aarjunsrinivasan/explorer_robot/blob/Week12_HW/License)
Autonomous exploration robot demo using turtlebot3


## Authors

**Arjun Srinivasan Ambalam** - [GitHub](https://github.com/aarjunsrinivasan)

## Overview
Roomba like vacuum cleaner that performs the basic functionality of autnomous navigation with obstacle avoidance.

## Description
An example of obstacle avoidance in ROS for a turtlebot simulated in Gazebo. The world here is turtlebot_world in which if any of the objects are moved and placed in front of the turtlebot, it will move around and start autonomously again. A launch file is made to start the nodes at once. Also, ROSBag is used to examine the output from the topics using an argument in launch file.

## Dependencies	
1. ROS Melodic - [Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Catkin(installed by default with ROS) - a low level build system macros and infrastructure for ROS.
3. Gazebo - [Gazebo Installation](http://gazebosim.org/tutorials?tut=ros_installing)
4. Turtlebot3 - [Turtlebot Package Installation](https://github.com/ROBOTIS-GIT/turtlebot3)
5. Turtlebot3 Gazebo package - [Turtlebot Package Installation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git)
6. ROS libraries - roscpp, geometry_msgs, sensor_msgs

## Build
Build using the following commands-

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
cd src/

git clone --recursive https://github.com/aarjunsrinivasan/explorer_robot
cd ..
catkin_make
```
This will make the workspace and package ready for execution

## To run the explorer_robot 
These are the following steps to run the explorer_robot.
1. Using launch file from a new terminal-
```
roslaunch explorer_robot demo.launch
```

## To record bag files
ROSBag recording can be done using the following command-
Open a terminal to run the launch file: 
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch explorer_robot demo.launch rosbagRecord:=true
```
When the flag(default = false) is true, it will record until SIGINT(ctrl+c) is pressed.

Following are the steps to examine the recorded bag file
1. Open a terminal to setup the Master Node: 
```
cd ~/catkin_ws
source ./devel/setup.bash
roscore
```

2. Open a new terminal to play the bag file:
Open the turtlebot_world in a terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```
```
cd ~/catkin_ws
source ./devel/setup.bash
cd src/explorer_robot/results
rosbag play recording.bag
```
The info on rosbag file can be seen using the following command
```
cd ~/catkin_ws/src/explorer_robot/results
rosbag info recording.bag
```
