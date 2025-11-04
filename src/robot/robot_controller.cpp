#include "robot_controller.h"

/*

    Things that need doing

    - write sample function properly
    - add fetch odometry
    - fix subs and pubs in general
    - write cmake function
    - add emergency return to home

*/

// clock for loop checking
using clock = std::chrono::steady_clock;  
using namespace std::chrono_literals;     


Robot::RobotController::RobotController(std::string serial_id, const std::shared_ptr<rclcpp::Node>& node) : 
    serial_id_(serial_id), 
    node_(node),
    emergency_(false),
    battery_(100),
    current_status_("IDLE"),
    registered_station_("")
{
    std::string cmd_topic = "/cmd_vel"; //In a multi robot setup this would be combined with serial_id as a prefix
    cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    std::string odom_topic = "/odometry/filtered"; //Filtered uses the IMU to correct drift (better for NAV2)
    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&Robot::RobotController::odomCallback, this, std::placeholders::_1)
    );

    soil_query_client_ = node_->create_client<unomas::srv::QuerySoil>("query_soil");

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

void Robot::RobotController::querySoil()
{
    if(registered_station_ != ""){
        last_query_odom_ = getOdometry();
        auto x = last_query_odom_.pose.pose.position.x;
        auto y = last_query_odom_.pose.pose.position.y;
        while(!soil_query_client_->wait_for_service(std::chrono::milliseconds(100))){
            RCLCPP_WARN(node_->get_logger(), "Waiting for soil query service to be available...");
        }
        auto request = std::make_shared<unomas::srv::QuerySoil::Request>();
        request->x = x;
        request->y = y;
        soil_query_client_->async_send_request(request,
            std::bind(&Robot::RobotController::soilRequestCallback, this, std::placeholders::_1)
        );
    }
}

void Robot::RobotController::soilRequestCallback(rclcpp::Client<unomas::srv::QuerySoil>::SharedFuture future)
{
    auto response = future.get();
    unomas::msg::SoilInfo soil_info_msg;
    soil_info_msg.x = last_query_odom_.pose.pose.position.x;
    soil_info_msg.y = last_query_odom_.pose.pose.position.y;
    soil_info_msg.moisture = response->moisture;
    soil_info_msg.nutrients = response->nutrients;
    soil_info_msg.ph = response->ph;
    soil_info_publisher_->publish(soil_info_msg);
}