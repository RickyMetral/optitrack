#include "rclcpp/rclcpp.hpp"
#include "VrpnClient.hpp"

int main(int argc, char** argv)
{
    	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VrpnClient>());
    	rclcpp::shutdown();
    	return 0;
}
