#ifndef BASE_STATION_STATUS_H
#define BASE_STATION_STATUS_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

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
    }; //For reference on states

    class BaseStationStatusManager : public rclcpp::Node
    {
        public:
            BaseStationStatusManager(std::string station_name);
            ~BaseStationStatusManager();
        
        private:
            void statusUpdatePubCallback();
            rclcpp::Publisher<unomas::msg::StatusUpdatePacket>::SharedPtr status_update_publisher_;

            void statusEmergencySubCallback(const std_msgs::msg::Bool::SharedPtr msg);
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscriber_;

            void statusBatterySubCallback(const std_msgs::msg::Int32::SharedPtr msg);
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_subscriber_;

            void statusPositionSubCallback(const geometry_msgs::msg::Point::SharedPtr msg);
            rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;

            void statusStateSubCallback(const std_msgs::msg::String::SharedPtr msg);
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscriber_;

            void statusTargetSubCallback(const geometry_msgs::msg::Point::SharedPtr msg);
            rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;

            rclcpp::TimerBase::SharedPtr ui_transmit_timer_;

            std::string name_;
            bool emergency_;
            int battery_;
            geometry_msgs::msg::Point current_position_;
            std::string current_state_;
            geometry_msgs::msg::Point target_position_;
    };
}





#endif // BASE_STATION_STATUS_H