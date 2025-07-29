#include "rclcpp/rclcpp.hpp"
#include "reset_estimator.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ResetEstimator>());
	rclcpp::shutdown();
	return 0;
}

