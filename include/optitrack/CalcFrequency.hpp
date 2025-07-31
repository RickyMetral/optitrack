/*
ROS2 Node to calculate the average frequency between messages received from VRPN
*/

#include <iostream>
#include <csignal>
#include <string>
#include <thread>
#include <vector>
#include <numeric>
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "rclcpp/rclcpp.hpp"

using std::vector;
using namespace std::chrono_literals;

class CalcFrequency : public rclcpp::Node {
public:
    CalcFrequency(std::string rigidBodyName, std::string server_address)
        : Node("calc_msg_frequency_node"), max_timestamps(5000)
    {
        const std::string full_address = rigidBodyName + "@" + server_address;
        this->connection = vrpn_get_connection_by_name(full_address.c_str());

        if (connection == nullptr) {
            std::cerr << "Failed to create VRPN connection object." << std::endl;
            exit(-1);
        }

        this->tracker = std::make_shared<vrpn_Tracker_Remote>(full_address.c_str(), connection);
        tracker->register_change_handler(this, &CalcFrequency::handle_pose);

        std::cout << "Attempting connection to VRPN server at: " << full_address << std::endl;

        for (int i = 0; i < 25; i++) {
            connection->mainloop();
            tracker->mainloop();
            std::this_thread::sleep_for(10ms);
        }

        if (connection->connected()) {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to VRPN server.");
        } else {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to VRPN server. Exiting...");
            raise(SIGINT);
        }

        rclcpp::on_shutdown([this]() {
            this->calcMsgFrequency();
        });

        this->timer_ = this->create_wall_timer(
            5ms, std::bind(&CalcFrequency::mainloop, this));
    }

    ~CalcFrequency()
    {
        this->tracker->unregister_change_handler(this, &CalcFrequency::handle_pose);
        delete this->connection;
    }

    double calcMsgFrequency()
    {
        // Calculate frequency if enough samples
        if (this->timestamps.size() >= 2) {
            vector<double> deltas;
            for (size_t i = 1; i < this->timestamps.size(); ++i) {
                deltas.push_back(this->timestamps[i] - this->timestamps[i - 1]);
            }

            double avg_interval_us = std::accumulate(deltas.begin(), deltas.end(), 0.0) / deltas.size();
            double frequency_hz = 1e6 / avg_interval_us;//1e6 converts microseconds -> seconds

            RCLCPP_INFO(this->get_logger(), "Average Delta Time: %f\nTracker frequency: %.2f Hz", 
			   avg_interval_us/1000, frequency_hz);

	    return frequency_hz;
        }
	return -1;
    }


private:
    vrpn_Connection* connection;
    std::shared_ptr<vrpn_Tracker_Remote> tracker;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<double> timestamps;
    size_t max_timestamps;

    static void VRPN_CALLBACK handle_pose(void* userData, const vrpn_TRACKERCB t)
    {
        CalcFrequency* self = static_cast<CalcFrequency*>(userData);

        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        double now_us = steady_clock.now().nanoseconds() / 1000.0;

        // Add timestamp
        self->timestamps.push_back(now_us);
        if (self->timestamps.size() > self->max_timestamps) {
            self->timestamps.erase(self->timestamps.begin());
        }

        // Optional: print quaternion and position
        RCLCPP_INFO(self->get_logger(), "Position: %.3f %.3f %.3f",
                    t.pos[0], t.pos[1], t.pos[2]);
    }

    void mainloop()
    {
        connection->mainloop();
        tracker->mainloop();
    }
};
