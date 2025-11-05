#include "robot_uplink.h"

Robot::RobotUplink::RobotUplink(std::string serial_id) :
    Node(serial_id),
    serial_id_(serial_id)
{
    controller_ = std::make_shared<RobotController>(serial_id_, shared_from_this());
}

Robot::RobotUplink::~RobotUplink()
{
    RCLCPP_INFO(this->get_logger(), "RobotUplink node for robot '%s' is shutting down.", serial_id_.c_str());
}

void Robot::RobotUplink::goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    geometry_msgs::msg::PoseArray goals = msg.poses;
    // for (const auto& pose : msg->poses) {
    //     goals.push_back(pose.position);
    // }
    controller_->setGoals(goals);
    RCLCPP_INFO(this->get_logger(), "Received %zu goals for robot '%s'.", goals.size(), serial_id_.c_str());
    controller_->autoNavigate();
}