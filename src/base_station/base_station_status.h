#ifndef BASE_STATION_STATUS_H
#define BASE_STATION_STATUS_H

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "unomas/msg/str_num.hpp"
#include "unomas/msg/str_str.hpp"
#include "unomas/msg/status_update_packet.hpp"

namespace BaseStation
{
    enum class State
    {
        IDLE,
        TRAVELLING,
        SURVEYING,
        ALLIGNING,
        SAMPLING,
        EMERGENCY
    };

    class BaseStationStatusManager : public rclcpp::Node
    {
        public:
            BaseStationStatusManager(std::string station_name);
            ~BaseStationStatusManager();
        
        private:
            void statusUpdatePubCallback();

            void statusEmergencySubCallback(const std_msgs::msg::Bool::SharedPtr msg);
            void statusBatterySubCallback(const std_msgs::msg::Int32::SharedPtr msg);
            void statusPositionSubCallback(const geometry_msgs::msg::Point::SharedPtr msg);
            void statusStateSubCallback(const std_msgs::msg::String::SharedPtr msg);
            void statusTargetSubCallback(const geometry_msgs::msg::Point::SharedPtr msg);

            bool emergency_;
            int battery_;
            geometry_msgs::msg::Point current_position_;
            State current_state_;
            geometry_msgs::msg::Point target_position_;
    };
}





#endif // BASE_STATION_STATUS_H