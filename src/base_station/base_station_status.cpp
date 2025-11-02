#include "base_station_status.h"

BaseStation::BaseStationStatusManager::BaseStationStatusManager(std::string station_name) : 
Node(station_name + "_status_manager"), 
name_(station_name),
emergency_(false),
battery_(100),
current_state_("IDLE")
{
    geometry_msgs::msg::Point init_point;
    init_point.x = 0.0;
    init_point.y = 0.0;
    init_point.z = 0.0;
    current_position_ = init_point;
    target_position_ = init_point;

    std::string emergency_topic = name_ + "/Status/Emergency";
    std::string battery_topic = name_ + "/Status/Battery";
    std::string position_topic = name_ + "/Status/Position";
    std::string state_topic = name_ + "/Status/State";
    std::string target_topic = name_ + "/Status/Target";

    ui_transmit_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&BaseStation::BaseStationStatusManager::statusUpdatePubCallback, this)
    );

    emergency_subscriber_ =
        this->create_subscription<std_msgs::msg::Bool>(
            emergency_topic, 10, std::bind(&BaseStation::BaseStationStatusManager::statusEmergencySubCallback, this, std::placeholders::_1)
        );

    battery_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32>(
            battery_topic, 10, std::bind(&BaseStation::BaseStationStatusManager::statusBatterySubCallback, this, std::placeholders::_1)
        );

    position_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Point>(
            position_topic, 10, std::bind(&BaseStation::BaseStationStatusManager::statusPositionSubCallback, this, std::placeholders::_1)
        ); 
    
    state_subscriber_ =
        this->create_subscription<std_msgs::msg::String>(
            state_topic, 10, std::bind(&BaseStation::BaseStationStatusManager::statusStateSubCallback, this, std::placeholders::_1)
        );

    target_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Point>(
            target_topic, 10, std::bind(&BaseStation::BaseStationStatusManager::statusTargetSubCallback, this, std::placeholders::_1)
        );

    status_update_publisher_ =
        this->create_publisher<unomas::msg::StatusUpdatePacket>(
            "/unomas/UpdatePacket", 10
        );
}

BaseStation::BaseStationStatusManager::~BaseStationStatusManager()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down BaseStationStatusManager node with name: '%s'", name_.c_str());
}

void BaseStation::BaseStationStatusManager::statusUpdatePubCallback()
{
    std::lock_guard<std::mutex> lock(resource_lock_);
    unomas::msg::StatusUpdatePacket status_msg;
    status_msg.name = name_;
    status_msg.emergency = emergency_;
    status_msg.battery = battery_;
    status_msg.current_position = current_position_;
    status_msg.current_state = current_state_;
    status_msg.target_position = target_position_;

    status_update_publisher_->publish(status_msg);
}

void BaseStation::BaseStationStatusManager::statusEmergencySubCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(resource_lock_);
    emergency_ = msg->data;
}

void BaseStation::BaseStationStatusManager::statusBatterySubCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(resource_lock_);
    battery_ = msg->data;
}

void BaseStation::BaseStationStatusManager::statusPositionSubCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(resource_lock_);
    current_position_ = *msg;
}

void BaseStation::BaseStationStatusManager::statusStateSubCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(resource_lock_);
    current_state_ = msg->data;
}

void BaseStation::BaseStationStatusManager::statusTargetSubCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(resource_lock_);
    target_position_ = *msg;
}