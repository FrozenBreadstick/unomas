#include "robot_controller.h"

Robot::RobotController::RobotController(std::string serial_id, const std::shared_ptr<rclcpp::Node>& node) : 
    serial_id_(serial_id), 
    node_(node),
    emergency_(false),
    battery_(100),
    current_status_("IDLE")
{
    std::string cmd_topic = "/cmd_vel"; //In a multi robot setup this would be combined with serial_id as a prefix
    cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    std::string odom_topic = "/odometry/filtered"; //Filtered uses the IMU to correct drift (better for NAV2)
    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&Robot::RobotController::odomCallback, this, std::placeholders::_1)
    );
}

Robot::RobotController::~RobotController()
{
    RCLCPP_INFO(node_->get_logger(), "RobotController for robot '%s' is shutting down.", serial_id_.c_str());
}

nav_msgs::msg::Odometry Robot::RobotController::getOdometry()
{
    return current_odometry_;
}

geometry_msgs::msg::Point Robot::RobotController::getGoal()
{
    return goal_position_; //Set goal position to the currently active goal being navigated to
}

void Robot::RobotController::sendCmd(geometry_msgs::msg::Twist cmd)
{
    cmd_publisher_->publish(cmd);
}

void Robot::RobotController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odometry_ = *msg;
}

void Robot::RobotController::setGoals(std::vector<geometry_msgs::msg::Point> goals)
{
    goals_ = goals;
}

bool Robot::RobotController::isEmergency()
{
    return emergency_;
}

std::string Robot::RobotController::getStatus()
{
    return current_status_;
}

int Robot::RobotController::getBattery()
{
    return battery_;
}

bool Robot::RobotController::autoNavigate()
{
    return true;
}