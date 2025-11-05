#include "base_station_register.h"

BaseStation::BaseStationRegister::BaseStationRegister(std::string station_name) : 
    Node(station_name), 
    station_name_(station_name)
{

    RCLCPP_INFO(this->get_logger(), "BaseStation Registration node started with name: '%s'", station_name_.c_str());

    serial_scan_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "Serial", 10, std::bind(&BaseStation::BaseStationRegister::serialScanCallback, this, std::placeholders::_1)
    );

    std::string registrar_topic = station_name_ + "/Registrar";
    std::string debug_response_topic = station_name_ + "/DebugResponse";

    registrar_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        registrar_topic, 10, std::bind(&BaseStation::BaseStationRegister::registrarCallback, this, std::placeholders::_1)
    );

    register_requester_publisher_ = this->create_publisher<unomas::msg::StrStr>(
        "Register", 10
    );

    path_planner_ = std::make_shared<BaseStation::BaseStationPath>(station_name_, shared_from_this());
}

BaseStation::BaseStationRegister::~BaseStationRegister()
{
    RCLCPP_INFO(this->get_logger(), "BaseStation Registration node is shutting down.");
}

void BaseStation::BaseStationRegister::serialScanCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received serial scan data: '%s'", msg->data.c_str());
    unomas::msg::StrStr register_msg;
    register_msg.textone = msg->data.c_str();
    register_msg.texttwo = station_name_;
    register_requester_publisher_->publish(register_msg);
    RCLCPP_INFO(this->get_logger(), "Sent registration request for robot: '%s'", msg->data.c_str());
}

void BaseStation::BaseStationRegister::registrarCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received registration confirmation for robot: '%s'", msg->data.c_str());
    if(std::find(registered_robots.begin(), registered_robots.end(), msg->data.c_str()) != registered_robots.end()) {
        RCLCPP_WARN(this->get_logger(), "Robot '%s' is already registered. Ignoring duplicate registration.", msg->data.c_str());
        return;
    }
    registered_robots.push_back(msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Total registered robots: %zu", registered_robots.size());
}