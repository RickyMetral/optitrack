#ifndef VRPN_CLIENT_H
#define VRPN_CLIENT_H

#include <string>
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace px4_msgs::msg;

class VrpnClient : public rclcpp::Node 
{
private:
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	VehicleOdometry odom_msg;
	vrpn_Connection* connection;//The connection object for vrpn server
	std::shared_ptr<vrpn_Tracker_Remote> tracker;//The vrpn tracker object

private:
	void set_msg(VehicleOdometry& msg);//Initializes default values for the odometry to be sent on callback
	static void VRPN_CALLBACK handle_pose(void* userData, const vrpn_TRACKERCB t);//When Pose data is received, it will publish to the vehcile_mocap_odometry topic
	static void VRPN_CALLBACK handle_vel(void* userData, const vrpn_TRACKERVELCB t);//When Pose data is received, it will publish to the vehcile_mocap_odometry topic


public:
	void rotate_quat_arr(double angle, char axis, double (&quat_arr)[4]);//Rotates quaternion array, by given amt of degrees along specified axis. Expexts quat_arr to be (X,Y,Z,W)
	void log_pose_tracker(const rclcpp::Logger& logger, const vrpn_TRACKERCB& t, const double (&quat)[4]);//Logger for the pose stream
	void log_vel_tracker(const rclcpp::Logger& logger, const vrpn_TRACKERVELCB& t);//Logger for the velocity stream
	VrpnClient(std::string rigid_body_name, std::string server_address); //Establishes Connection and registers callbacks
	VrpnClient(); //Establishes Connection and registers callbacks. Uses parameter for rigid_body_name and server_address(DEFAULTS: rigid_body_name = 'starling_2', server_address = '192.168.1.42:3883')
	~VrpnClient();//Unregisters the tracker callbacks
	void mainloop();//One iteration of connection and tracker mainloop and sleeps for 10ms
};

#endif //VRPN_CLIENT_H
