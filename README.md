# OptiTrack ROS2 Package

A ROS2 package designed to publish VRPN data for offboard indoor flight control using OptiTrack motion capture system with the Starling 2 drone at AIMSLab.

## Overview

This package enables external position estimation for PX4-based drones using OptiTrack motion capture data. It supports VRPN streaming and optionally includes UDP connection capabilities for data streaming from virtual machines.
Optionally includes my UDP conneciton class if you want to stream the data from the VM over UDP instead of using VRPN.

This package contains three nodes and a launch file: 

1. publish_mocap_odometry_node
   - Receives the VRPN data
   - Sends received data to starling 2 ekf2
   - Has a 5ms sleep between callbacks
     
2. reset_estimator_node:
   - Necessary before any flight using motive
   - Sends a reset flag to EKF2 and sensors
   - Waits for the EKF2 to converge to a stable measurement
   - Terminates on convergence
  
3. calc_msg_frequency_node:
   - Receives VRPN data and measures the frequency it receives the pose estimates
   - Runs until stopped and averages all the timestamps
4. recv_mocap_launch:
   - Use this when you are going to fly the drone using mocap
   - Launches pubslish_mocap_odometry_node and reset_estimator_node
   - Logs when estimator has converged 

## Features

- **VRPN Integration**: Streams OptiTrack motion capture data via VRPN protocol
- **PX4 Compatibility**: Designed for external position estimation with PX4 autopilot
- **UDP Streaming**: Optional UDP connection class to strema from AIMSLab VM
- **Indoor Flight**: Enables precise offboard control in indoor environments

## Dependencies

### Required ROS2 Packages
- `px4_ros_com` - PX4 ROS2 communication interface
- `px4_msgs` - PX4 message definitions

### System Dependencies
- ROS2 (Foxy recommended)
- VRPN library
- CMake >= 3.5
- C++17 compiler


## Installation

### 1. Install System Dependencies

```bash

# Install ROS2 development tools
sudo apt install ros-$ROS_DISTRO-ament-cmake
sudo apt install ros-$ROS_DISTRO-rclcpp
sudo apt install ros-$ROS_DISTRO-std-msgs
sudo apt install ros-$ROS_DISTRO-geometry-msgs
```

### 2. Install PX4 Dependencies

```bash
# Clone and build px4_ros_com
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git

# Build dependencies
cd ~/ros2_ws
colcon build --packages-select px4_msgs px4_ros_com
source install/setup.bash
```

### 3. Clone and Build OptiTrack Package

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the OptiTrack package
git clone https://github.com/RickyMetral/optitrack.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select optitrack
source install/setup.bash
```

## Configuration

### OptiTrack Motive Setup

1. **Start Motive Software** by opening the app on the lab pc. Refer to optitrack docs here:
   - [https://0406hockey.gitbook.io/mocap-clover](url)
   - [https://docs.optitrack.com/motive](url)
   - If you have further questions contact one of Dr. Lauf's students, me or Dr. Lauf
   
2. **Create a Rigid Body**:
   - Select markers for your drone
   - Create a rigid body
   - Ensure the rigid body is actively tracked
   - [https://www.youtube.com/watch?v=Z9kO7jJgCLE](url)
     
3. **Select Rigid Bodies**
   - In the top right of motive navigate to the assets tab and select the rigid body you want to track

### Basic Launch

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch the OptiTrack node
ros2 run optitrack publish_mocap_odometry_node

# Or with parameters
ros2 run optitrack publish_mocap_odometry_node --ros-args -p server_address:=192.168.1.100 -p rigid_body_name:=drone_01
```

### Using Launch File

Follow the same build and sourcing instructions as abobe but instead run:

```bash

ros2 launch optitrack recv_mocap_launch.py rb_name:=<rigid_body_name>

#Or run using default ridid body (starling_2)
ros2 launch optitrack recv_mocap_launch.py
```

### Verify Data Stream

```bash
# Check published topics
ros2 topic list

# Monitor pose data
ros2 topic echo /fmu/in/vehicle_visual_odometry
```

## Troubleshooting

### Common Issues

1. **No VRPN Connection**:
   - Verify OptiTrack PC IP address
   - Check firewall settings
   - Ensure VRPN streaming is enabled in Motive

2. **No Rigid Body Data**:
   - Verify rigid body name matches configuration
   - Check if rigid body is actively tracked in Motive
   - Ensure sufficient markers are visible

3. **Poor Tracking Quality**:
   - Recalibrate OptiTrack system
   - Check the markers for any damage
  
4. **PX4 Integration Issues**:
   - Verify EKF2 parameters are set correctly
   - Check coordinate frame transformations
   - Monitor `/fmu/in/vehicle_visual_odometry` topic

### Debug

```bash
# Check ROS2 node status
ros2 node list
ros2 node info /publish_mocap_odometry_node
```

## Topics

### Published Topics

- `/fmu/in/vehilce_visual_odometry` (px4_msgs/msg/VehicleOdometry): publish_mocap_odometry_node


### Subscribed Topics

- `/fmu/out/vehicle_odometry` (px4_msgs/msg/VehicleOdometry): reset_estimator_node


## Reference Documentation

- [PX4 External Position Estimation](https://docs.px4.io/v1.12/en/ros/external_position_estimation.html)
- [OptiTrack Motive Documentation](https://docs.optitrack.com/)
- [VRPN Documentation](https://github.com/vrpn/vrpn)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request
