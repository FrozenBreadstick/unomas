#ifndef ROBOT_UPLINK_H
#define ROBOT_UPLINK_H

#include "rclcpp/rclcpp.hpp"
#include "robot_controller.h" 

#include "geometry_msgs/msg/pose_array.hpp"

namespace Robot {
    class RobotUplink : public rclcpp::Node
    {
        public:
            RobotUplink(std::string serial_id);
            ~RobotUplink();

        private:
            std::shared_ptr<Robot::RobotController> controller_;
            
            // Will publish all the stuff that Base Station Status reads.
            
            // SUbscribes to waht is published by the Base Station Path node
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_subscriber_;
            void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

            std::string serial_id_;
    };
}

#endif // ROBOT_UPLINK_H