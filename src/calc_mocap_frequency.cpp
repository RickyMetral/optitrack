#include "CalcFrequency.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    if(argc != 2){
	    std::cerr <<  "Usage: ros2 run optitrack calc_mocap_frequency_node <rigid_body_name\n";
        return 1;
    }

    std::string rigidBody = argv[1];
    std::string ip_addr = "192.168.1.42:3883";

    auto node = std::make_shared<CalcFrequency>(rigidBody, ip_addr);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
