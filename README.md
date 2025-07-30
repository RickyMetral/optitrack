# VRPN Client ROS2 Package for Starling 2

This is a ROS2 package to publish VRPN data in ROS2 on the Starling 2 drone at aimslab with Optitrack. This package allows for offboard indoor flight using Mocap

Optionally includes my UDP conneciton class if you want to stream the data from the VM over UDP instead of using VRPN.

PX4 Docs for using external pose esitmation:
  [https://docs.px4.io/v1.12/en/ros/external_position_estimation.html](url)
  
Dependencies:
px4_ros_com
px4_msgs
