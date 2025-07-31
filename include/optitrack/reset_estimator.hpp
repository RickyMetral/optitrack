#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <csignal>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>

#define CONVERGE_THRESHOLD 0.001

using std::max;
using std::min;
using std::vector;

class ResetEstimator : public rclcpp::Node {
private:
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::thread convergence_thread;//Thread to check if EKF2 has converged
	//Queues for the vals we receive from ekf2
	vector<float> x_values;
	vector<float> y_values;
	vector<float> z_values;

	std::atomic<bool> stop_thread{false};//Bool to check if thread is finished

private:
	//ROS2 Callback to update our the values in our queues
	void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
		if(this->stop_thread.load()){
			this->convergence_thread.join();
			RCLCPP_INFO(this->get_logger(), "Convergence Complete. Raising SIGINT");
			raise(SIGINT);
			return;
		}
		x_values.push_back(msg->position[0]);
		y_values.push_back(msg->position[1]);
		z_values.push_back(msg->position[2]);

		//RCLCPP_INFO(this->get_logger(), "New Position: %.3f, %.3f, %.3f",
			   //msg->position[0], msg->position[1], msg->position[2]);

		if(x_values.size() > 30) {x_values.erase(x_values.begin());}
		if(y_values.size() > 30) {y_values.erase(y_values.begin());}
		if(z_values.size() > 30) {z_values.erase(z_values.begin());}

	}

	//Sends reset flag EKF2 sensors to allow convergence with mocap odometry
	//Starts thread to check for convergence
	void send_reset_flag(){
		auto msg = px4_msgs::msg::VehicleCommand();
		msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
		msg.param1 = 1.0f;// Tells cmd to reboot sensors and estimator
		msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
		msg.target_system = 1;//PX4 flight controller
		msg.target_component = 1;//Autopilot component
		msg.source_system = 1;//Companion Computer
		msg.source_component = 1;//Companion system
		msg.from_external = true;

		this->command_pub_->publish(msg);
	
		RCLCPP_INFO(this->get_logger(), 
			"Sent estimate reset command to PX4(VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN : 1)");
		//If the thread is already running don't run again
		if(convergence_thread.joinable()){
			return;
		}
      		std::this_thread::sleep_for(std::chrono::seconds(1));
		this->convergence_thread = std::thread(&ResetEstimator::wait_estimator_converge, this);
	}
	//Checks our queues and makes sure EKF2 variance
	//is beneath suitable threshold(CONVERGENCE_THRESHOLD)
	void wait_estimator_converge(){
		bool converged = false;

		RCLCPP_INFO(this->get_logger(), "Waiting for the estimator to converge...");	
		while(rclcpp::ok() && !this->stop_thread.load()){
			//Grabs min and max val from our queues
			auto [min_x, max_x] = std::minmax_element(x_values.begin(), x_values.end());
		    	auto [min_y, max_y] = std::minmax_element(y_values.begin(), y_values.end());
		    	auto [min_z, max_z] = std::minmax_element(z_values.begin(), z_values.end());
			
			//Calculates our variance
			float range_x = *max_x - *min_x;
			float range_y = *max_y - *min_y;
			float range_z = *max_z - *min_z;

			//RCLCPP_INFO(this->get_logger(),
				//"range_x: %.3f range_y: %.3f range_z: %.3f",
			       	//range_x, range_y, range_z);
			
			//Checks if variances are beneath threshold
			if(range_x < CONVERGE_THRESHOLD && range_y < CONVERGE_THRESHOLD && 
			   range_z < CONVERGE_THRESHOLD) {
				this->stop_thread.store(true);
				converged = true; 
				break;
			}
		}
		if(converged){
			RCLCPP_INFO(this->get_logger(), "Estimator Converged Successfully.");
		}
		return;
	}

public:
	~ResetEstimator(){
		this->stop_thread.store(true);
		if(convergence_thread.joinable()){
			convergence_thread.join();
		}
	}

	ResetEstimator() : Node("estimator_reset_node") {
		this->x_values = vector<float>(30, 1000);
		this->y_values = vector<float>(30, 1000);
		this->z_values = vector<float>(30, 1000);

		//To avoid immediate convergence
		x_values.push_back(-1000);
		y_values.push_back(-1000);
		z_values.push_back(-1000);

		this->command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
		  "/fmu/in/vehicle_command", 10);
		this->odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		  "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
		  std::bind(&ResetEstimator::odom_callback, this, std::placeholders::_1));

		this->timer_ = this->create_wall_timer(
			std::chrono::milliseconds(500),
			[this]() {
				//We only want to send reset flag once so we cancel
				this->timer_->cancel();
				this->send_reset_flag();
			}
		);
	}
};
