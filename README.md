# ENPH353 Lab 3 for students

This repository contains the ROS2 workspace for Lab 3 for ENPH353 course taught at University of British Columbia.

The ROS2 packages within the src/ folder are:

| Packages | Description |
| :--- | :--- |
| custom_plugins | Contains a C++ Gazebo world level plugin that resets non-static entities by teleporting them back to their spawnpoints. |
| line driver_pkg | Contains the script and launch file to get the robot to follow the line. |
| my_robot_description | Contains a differential drive URDF robot that students will control by creating their own ROS2 package. Also contains an RViz launch file to enable viewing the robot.  |
| system_start_pkg | Contains the launch file and gazebo-to-ROS2 config files. |
| world_pkg | Describes the monza simulation world and its associated assets. |
