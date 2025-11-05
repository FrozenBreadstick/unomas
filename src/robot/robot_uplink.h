#ifndef ROBOT_UPLINK_H
#define ROBOT_UPLINK_H

#include "rclcpp/rclcpp.hpp"
#include "robot_controller.h" 

#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "unomas/msg/str_str.hpp"
#include "unomas/msg/str_num.hpp"

#include "unomas/msg/addressed_pose_array.hpp"

namespace Robot {
    class RobotUplink : public rclcpp::Node
    {
        public:
            RobotUplink(std::string serial_id);
            ~RobotUplink();

            void initialise();

        private:
            std::shared_ptr<Robot::RobotController> controller_;
            
            void UplinkTimerCallback();
            
            // SUbscribes to waht is published by the Base Station Path node
            rclcpp::Subscription<unomas::msg::AddressedPoseArray>::SharedPtr goal_subscriber_;
            void goalCallback(const unomas::msg::AddressedPoseArray::SharedPtr msg);

            std::string serial_id_;

            void registerReceiverCallback(const unomas::msg::StrStr::SharedPtr msg);

            rclcpp::TimerBase::SharedPtr uplink_timer_;

            rclcpp::Subscription<unomas::msg::StrStr>::SharedPtr register_receiver_subscriber_;

            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_broadcast_publisher_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr register_serial_publisher_;

            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_publisher_;
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_publisher_;
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_publisher_;

            std::string registered_station_;
            bool registered_;

            bool emergency_;
            int battery_;
            geometry_msgs::msg::Point position_;
            std::string state_;
            geometry_msgs::msg::Point target_;

    };
}

#endif // ROBOT_UPLINK_H