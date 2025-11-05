#include "robot_uplink.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto params = std::make_shared<rclcpp::Node>("params_robot");

    params->declare_parameter<std::string>("serial_id");
    std::string serial_id;
    params->get_parameter("serial_id", serial_id);

    auto robot = std::make_shared<Robot::RobotUplink>(serial_id);

    robot->initialise();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot);   
    executor.spin();

    return 0;
}