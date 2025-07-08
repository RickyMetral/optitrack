#ifndef VRPN_CLIENT_H
#define VRPN_CLIENT_H

#include <string>
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace px4_msgs::msg;

class VrpnClient
{
private:
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_publisher_;
	rclcpp::Node::SharedPtr node;
	VehicleOdometry odom_msg;
	vrpn_Connection* connection;//The connection object for vrpn server
	std::shared_ptr<vrpn_Tracker_Remote> tracker;//The tracker vrpn object
	//vrpn_Tracker_Remote tracker;//The tracker vrpn object
private:
	void setMsg(VehicleOdometry& msg);//Initializes default values for the odometry to be sent on callback
	static void VRPN_CALLBACK handle_pose(void* userData, const vrpn_TRACKERCB t);//When Pose data is received, it will publish to the vehcile_mocap_odometry topic
	static void VRPN_CALLBACK handle_vel(void* userData, const vrpn_TRACKERVELCB t);//When Pose data is received, it will publish to the vehcile_mocap_odometry topic

public:
	VrpnClient(std::string rigidBodyName, std::string server_address); //Establishes Connection and registers callbacks
	~VrpnClient();//Unregisters the tracker callbacks
	void mainloop();//One iteration of connection and tracker mainloop and sleeps for 10ms
};

#endif //VRPN_CLIENT_H
