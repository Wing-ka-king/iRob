# Introduction-to-Robotics-DD2410-
Assignment work in KTH course- Introduction to Robotics.

## Assignment 1: ROS Taster
This assignment allowed students to get familiarized with ros environment. The objective was to make a wall follower using Kobuki robot simulation. This was divided into three tasks:

- To make an open loop controller. 
- To make a cartesian controller (based on differential drive model) to make the robot go in a straight line
- To add a wall follower on top of cartesian controller to make the robot follow the wall.

## How to execute
Create a catkin workspace

`mkdir ~/catkin_ws && mkdir ~/catkin_ws/src`

Git clone this repo in src and then:

`catkin_make`

To execute task 2: 

`roslaunch ras_lab1_launch lab1_task2.launch`

To execute task 3:

`roslaunch ras_lab1_launch lab1_task3.launch`

## Assignment 2: Inverse Kinematics
In this assignment you will program the inverse kinematic algorithm for a robot, which will move its joints so that it follows a desired path with the end-effector. 

It is composed of two parts:

A 3 DOF scara robot, with an inverse kinematic solution that is possible in analytic form. Running instructions: 

`roslaunch kinematics_assignment scara_launch.launch`

A 7 DOF kuka robot, with the inverse kinematic solution to be implemented with iterative algorithms:

`roslaunch kinematics_assignment kuka_launch.launch`

## Assignment 3: Path Planning for Dubin's car
See this for instructions: https://cisprague.github.io/dubins/#/

## Assignment 4: Mapping
In this assignment we learnt about occupancy grid mapping. Running instructions: 

Terminal 1: `roscore`
Terminal 2: `rosrun mapping_assignment main.py FILE` 

where FILE is a rosbag file 

## Assignment 5: Robot State Machine
Integrating all above ideas to get a complete robotic system which can pick up a cube and transfer it to another location in the room!
