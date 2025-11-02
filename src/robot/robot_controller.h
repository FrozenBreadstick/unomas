#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace Robot {
    class RobotController
    {
        public:
            RobotController(std::string serial_id, const std::shared_ptr<rclcpp::Node>& node);
            ~RobotController();

            nav_msgs::msg::Odometry getOdometry();

            geometry_msgs::msg::Point getGoal();

            std::string getStatus();

            bool isEmergency();

            int getBattery();

            void sendCmd(geometry_msgs::msg::Twist cmd);

            void setGoals(std::vector<geometry_msgs::msg::Point> goals);

            bool autoNavigate(); // Returns true if vector of goals exists, function to use NAV2 to move between goals in goals_ vector
            // Use threads so that it doesnt block the main uplink nodes subscribers and publishers

        private:
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            std::string serial_id_;
            const std::shared_ptr<rclcpp::Node>& node_;
            std::vector<geometry_msgs::msg::Point> goals_;
            nav_msgs::msg::Odometry current_odometry_;
            bool emergency_;
            int battery_;
            std::string current_status_;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

            geometry_msgs::msg::Point goal_position_;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    };
}

#endif // ROBOT_INTERFACE_H