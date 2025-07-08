#include "rclcpp/rclcpp.hpp"
#include "VrpnClient.hpp"

int main(int argc, char** argv)
{
	if (argc < 2) {
		std::cerr << "Usage: vrpn_node <rigid_body_name>" << std::endl;
		return -1;
	}
	std::string rigidBody = argv[1];
	std::string server_ip = "192.168.1.42:3883";

    	rclcpp::init(argc, argv);

    	auto vrpn = std::make_shared<VrpnClient>(rigidBody, server_ip); 

    	while (rclcpp::ok()){
    	        vrpn->mainloop();
	}
    	rclcpp::shutdown();
    	return 0;
}
