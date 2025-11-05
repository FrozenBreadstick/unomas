#ifndef BASE_STATION_REGISTER_H
#define BASE_STATION_REGISTER_H

#include "rclcpp/rclcpp.hpp"
#include "base_station_path.h"
#include "std_msgs/msg/string.hpp"
#include "unomas/msg/str_num.hpp"
#include "unomas/msg/str_str.hpp"

namespace BaseStation
{
    class BaseStationRegister : public rclcpp::Node
    {
        public:
            BaseStationRegister(std::string station_name);
            ~BaseStationRegister();

            void initialise();
            
        private:
            std::shared_ptr<BaseStation::BaseStationPath> path_planner_;
            std::string station_name_;

            void serialScanCallback(const std_msgs::msg::String::SharedPtr msg);

            void registrarCallback(const std_msgs::msg::String::SharedPtr msg);

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_scan_subscriber_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr registrar_subscriber_;

            rclcpp::Publisher<unomas::msg::StrStr>::SharedPtr register_requester_publisher_;

            std::vector<std::string> registered_robots;
    };
}

#endif // BASE_STATION_REGISTER_H