#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "VrpnClient.hpp"

using namespace px4_msgs::msg;


std::ostream& operator<<(std::ostream& os, const vrpn_TRACKERCB t)
{
   os << "Quaternions: " << t.quat[3] << " " << t.quat[0]
   << " " <<  t.quat[1] << " " << t.quat[2] << "\n"
   "Position: " << t.pos[0] << " "  <<  t.pos[1] << " " << t.pos[2] << std::endl;
   return os;
}

std::ostream& operator<<(std::ostream& os, const vrpn_TRACKERVELCB t)
{
    os << "Velocity Quat: " << t.vel_quat[3] << " " << t.vel_quat[0] 
        <<  t.vel_quat[1] << " " << t.vel_quat[2] << "\n"
        "Velocity: " << t.vel[0] << " " <<  t.vel[1] << " " << t.vel[2]  << std::endl;
    return os;
}

void VrpnClient::setMsg(VehicleOdometry& msg)
{
   //Defines our coordinate frames
   msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
   msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
   
   msg.reset_counter = 0;
   msg.quality = 100;
   //These are optional covariance matrices
   msg.position_variance[0] = NAN;
   msg.orientation_variance[0] = NAN;
   msg.velocity_variance[0] = NAN;
}

//Callback function for receiving tracker velocity data
void VRPN_CALLBACK VrpnClient::handle_vel(void* userData, const vrpn_TRACKERVELCB t)
{
  VrpnClient* self = static_cast<VrpnClient*> (userData);

  self->odom_msg.velocity[0] = t.vel[0];//(x,y,z), motive has Y as vertical axis
  self->odom_msg.velocity[1] = t.vel[2];
  self->odom_msg.velocity[2] = -t.vel[1];

  self->odom_msg.angular_velocity[0] = t.vel_quat[3];//(w,x,y,z)
  self->odom_msg.angular_velocity[1] = t.vel_quat[0];
  self->odom_msg.angular_velocity[2] = t.vel_quat[1];
  self->odom_msg.angular_velocity[3] = t.vel_quat[2];
}

//Callback function for receiving tracker position data
void VRPN_CALLBACK VrpnClient::handle_pose(void* userData, const vrpn_TRACKERCB t)
{
    VrpnClient* self = static_cast<VrpnClient*> (userData);

    self->odom_msg.position[0] = t.pos[0];//(x,y,z), motive has Y as vertical axis
    self->odom_msg.position[1] = t.pos[2];
    self->odom_msg.position[2] = -t.pos[1];

    self->odom_msg.q[0] = t.quat[3];//(w,x,y,z)
    self->odom_msg.q[1] = t.quat[0];
    self->odom_msg.q[2] = t.quat[1];
    self->odom_msg.q[3] = t.quat[2];
    self->odom_publisher_->publish(self->odom_msg);
    std::cout << t << std::endl;
}



VrpnClient::VrpnClient(std::string rigidBodyName, std::string server_address)
{
    // Set your tracker name & VRPN server address
    const std::string full_address = rigidBodyName + "@" + server_address;
    setMsg(odom_msg);
    this->node = rclcpp::Node::make_shared("publish_motive_odometry_node");
    this->odom_publisher_ = node->create_publisher<VehicleOdometry>("/fmu/in/vehicle_mocap_odometry", 10); 

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
    tracker->register_change_handler(this, &VrpnClient::handle_vel);

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
        std::cout << "Successfully connected to VRPN server." << std::endl;
    } else {
        std::cerr << "Failed to connect to VRPN server after waiting." << std::endl;
	exit(-1);
    }
}

VrpnClient::~VrpnClient()
{

	this->tracker->unregister_change_handler(this, &VrpnClient::handle_pose);
	this->tracker->unregister_change_handler(this, &VrpnClient::handle_vel);
	delete this->connection;
}

void VrpnClient::mainloop()
{
	connection->mainloop();
	tracker->mainloop();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
