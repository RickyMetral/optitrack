#include <iostream>
#include <csignal>
#include <math.h>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "rclcpp/rclcpp.hpp"
#include "VrpnClient.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace px4_msgs::msg;


void VrpnClient::log_pose_tracker(const rclcpp::Logger& logger, const vrpn_TRACKERCB& t, const double (&quat)[4])
{
   RCLCPP_INFO(logger,
   "Quaternions: %.3f %.3f %.3f %.3f\nPosition: %.3f %.3f %.3f",
   quat[3], quat[0], quat[1], quat[2],
   t.pos[0], t.pos[1], t.pos[2]
   );
}

void log_changed_pose(const rclcpp::Logger& logger, const vrpn_TRACKERCB& t, const double (&quat)[4])
{
   RCLCPP_INFO(logger,
   "Quaternions Before Transform: %.3f %.3f %.3f %.3f\nQuaternions After Transform: %.3f %.3f %.3f %.3f\nNED Position: %.3f %.3f %.3f",
   t.quat[3], t.quat[0], t.quat[1], t.quat[2],
   quat[3], quat[0], quat[2], -quat[1],
   t.pos[0], t.pos[2], -t.pos[1]
   );
}

void VrpnClient::log_vel_tracker(const rclcpp::Logger& logger, const vrpn_TRACKERVELCB& t)
{
   RCLCPP_INFO(logger,
   "Velocity Quaternions: %.3f %.3f %.3f %.3f\nVelocity: %.3f %.3f %.3f",
   t.vel_quat[3], t.vel_quat[0], t.vel_quat[1], t.vel_quat[2],
   t.vel[0], t.vel[1], t.vel[2]
   );
}


void VrpnClient::rotate_quat_arr(double angle, char axis, double(&quat_arr)[4])
{
   //Transform our array into tf2 
   tf2::Quaternion q(quat_arr[0], quat_arr[1], quat_arr[2], quat_arr[3]);//(X, Y, Z, W)
   
   constexpr double DEG2RAD = M_PI/ 180.0;//Conversion constant
   double rotate_rad = angle * DEG2RAD;//Converts angle to radians 

   tf2::Quaternion q_transform; 

   //Decdiing which axis to rotate around
   if(axis == 'X' ||  axis == 'x')
   {
   	q_transform.setRPY(rotate_rad, 0, 0);
   }
   else if(axis == 'Y' ||  axis == 'y')
   {
   	q_transform.setRPY(0, rotate_rad, 0);
   }
   else if(axis == 'Z' ||  axis == 'z')
   {
   	q_transform.setRPY(0, 0, rotate_rad);
   } 
   else
   {
	std::cerr << "Invalid Rotation Axis Provided" << std::endl;
   }

   tf2::Quaternion res = q_transform * q;//Resultant vector
   res.normalize();
   
   quat_arr[0] = static_cast<double>(res.x());
   quat_arr[1] = static_cast<double>(res.y());
   quat_arr[2] = static_cast<double>(res.z());
   quat_arr[3] = static_cast<double>(res.w());
} 

void VrpnClient::set_msg(VehicleOdometry& msg)
{
   //Defines our coordinate frames
   msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
   msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;

   msg.reset_counter = 0;
   msg.quality = 100;
   
   //These are optional covariance matrices
   msg.position_variance = {0.0001, 0.0001, 0.001};
   msg.orientation_variance = {0.0001, 0.0001, 0.001};
   msg.velocity_variance[0] = NAN;

}

//Callback function for receiving tracker velocity data
void VRPN_CALLBACK VrpnClient::handle_vel(void* userData, const vrpn_TRACKERVELCB t)
{
   VrpnClient* self = static_cast<VrpnClient*> (userData);

   double quat_arr[4] = {t.vel_quat[0], t.vel_quat[1], t.vel_quat[2], t.vel_quat[3]};

   rclcpp::Clock steady_clock(RCL_STEADY_TIME);//Time since boot
   self->odom_msg.timestamp = steady_clock.now().nanoseconds() / 1000;//timestamp is the current time since boot
   self->odom_msg.timestamp_sample = self->odom_msg.timestamp - 10;//Timestamp_sample is when sample was taken from mocap


   self->odom_msg.velocity[0] = t.vel[0];//(x,y,z), motive has Y as vertical axis
   self->odom_msg.velocity[1] = t.vel[2];
   self->odom_msg.velocity[2] = -t.vel[1];

   self->rotate_quat_arr(90, 'x', quat_arr);

   self->odom_msg.angular_velocity[0] = quat_arr[3];//(w,x,y,z)
   self->odom_msg.angular_velocity[1] = quat_arr[0];
   self->odom_msg.angular_velocity[2] = quat_arr[1];
   self->odom_msg.angular_velocity[3] = quat_arr[2];

   self->log_vel_tracker(self->get_logger(), t);
}

//Callback function for receiving tracker position data
void VRPN_CALLBACK VrpnClient::handle_pose(void* userData, const vrpn_TRACKERCB t)
{
    VrpnClient* self = static_cast<VrpnClient*> (userData);

    double quat_arr[4] = {t.quat[0], t.quat[1], t.quat[2], t.quat[3]};

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);//Time since boot
    self->odom_msg.timestamp = steady_clock.now().nanoseconds() / 1000;//timestamp is the current time since boot
    self->odom_msg.timestamp_sample = self->odom_msg.timestamp - 10;//Timestamp_sample is when sample was taken from mocap


    self->odom_msg.position[0] = t.pos[0];//Motive x -> NED X(North)
    self->odom_msg.position[1] = t.pos[2];//Motive z -> NED Y(EAST)
    self->odom_msg.position[2] = -t.pos[1];//Motive y -> NED Z(Down)


    //Changes Motive coordinate frame (x-left, z-forward, y-up) to voxl2 frame (NED)
    //(x, y, z, w) -> (w, x, y, z)
    self->odom_msg.q[0] = quat_arr[3];//w unchanged
    self->odom_msg.q[1] = quat_arr[0];//Motive x -> NED X(North)
    self->odom_msg.q[2] = quat_arr[2];//Motive z -> NED Y(East)
    self->odom_msg.q[3] = -quat_arr[1];//Motive y -> NED Z(Down)

    self->odom_msg.reset_counter = 0;
    self->odom_msg.quality = 100;

    self->odom_publisher_->publish(self->odom_msg);
    self->log_pose_tracker(self->get_logger(), t, quat_arr);
    //log_changed_pose(self->get_logger(), t, quat_arr);
}



VrpnClient::VrpnClient(std::string rigidBodyName, std::string server_address) : Node("publish_mocap_odometry_node")
{
    // Set your tracker name & VRPN server address
    const std::string full_address = rigidBodyName + "@" + server_address;
    set_msg(odom_msg);//Sets the default parameters for our message
    this->odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10); 

    // Use vrpn_get_connection_by_name() instead of implicit connection
    this->connection = vrpn_get_connection_by_name(full_address.c_str());

    if (connection == nullptr) {
        std::cerr << "Failed to create VRPN connection object." << std::endl;
	exit(-1);
    }

    // Create Tracker client attached to existing connection
    //this->tracker = std::make_shared<vrpn_Tracker_Remote>(full_address.c_str(), connection);
    this->tracker = std::make_shared<vrpn_Tracker_Remote>(full_address.c_str(), connection);

    // Register callback function
    tracker->register_change_handler(this, &VrpnClient::handle_pose);
    //tracker->register_change_handler(this, &VrpnClient::handle_vel);

    std::cout << "Attempting connection to VRPN server at: " << full_address << std::endl;

    // Allow time for connection handshake
    const int initial_wait_iterations = 25;
    for (int i = 0; i < initial_wait_iterations; i++) {
        connection->mainloop();
        tracker->mainloop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Verify connection
    if (connection->connected()) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to VRPN server.");
    } else {
       	RCLCPP_FATAL(this->get_logger(), "Failed to connect to VRPN server after waiting. Exiting...");
	raise(SIGINT);
    }
    this->timer_ = this->create_wall_timer(
		   std::chrono::milliseconds(10),
		   std::bind(&VrpnClient::mainloop, this));

}

VrpnClient::VrpnClient() : Node("publish_mocap_odometry_node")
{
    // Set your tracker name & VRPN server address
    this->declare_parameter<std::string>("rigid_body_name", "starling_2");
    this->declare_parameter<std::string>("server_address", "192.168.1.42:3883");

    std::string rigid_body_name = this->get_parameter("rigid_body_name").as_string();
    std::string server_address = this->get_parameter("server_address").as_string();

    const std::string full_address = rigid_body_name + "@" + server_address;
    set_msg(odom_msg);//Sets the default parameters for our message
    this->odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10); 

    // Use vrpn_get_connection_by_name() instead of implicit connection
    this->connection = vrpn_get_connection_by_name(full_address.c_str());

    if (connection == nullptr) {
        std::cerr << "Failed to create VRPN connection object." << std::endl;
	exit(-1);
    }

    // Create Tracker client attached to existing connection
    //this->tracker = std::make_shared<vrpn_Tracker_Remote>(full_address.c_str(), connection);
    this->tracker = std::make_shared<vrpn_Tracker_Remote>(full_address.c_str(), connection);

    // Register callback function
    tracker->register_change_handler(this, &VrpnClient::handle_pose);
    //tracker->register_change_handler(this, &VrpnClient::handle_vel);

    std::cout << "Attempting connection to VRPN server at: " << full_address << std::endl;

    // Allow time for connection handshake
    const int initial_wait_iterations = 25;
    for (int i = 0; i < initial_wait_iterations; i++) {
        connection->mainloop();
        tracker->mainloop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Verify connection
    if (connection->connected()) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to VRPN server.");
    } else {
       	RCLCPP_FATAL(this->get_logger(), "Failed to connect to VRPN server after waiting. Exiting...");
	raise(SIGINT);
    }
    this->timer_ = this->create_wall_timer(
		   std::chrono::milliseconds(10),
		   std::bind(&VrpnClient::mainloop, this));

}

VrpnClient::~VrpnClient()
{
   this->tracker->unregister_change_handler(this, &VrpnClient::handle_pose);
   //this->tracker->unregister_change_handler(this, &VrpnClient::handle_vel);
   delete this->connection;
}

void VrpnClient::mainloop()
{  
   connection->mainloop();
   tracker->mainloop();
}
