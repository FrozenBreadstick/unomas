#include "robot_uplink.h"

Robot::RobotUplink::RobotUplink(std::string serial_id) :
    Node(serial_id + "_" + std::to_string(getpid())), 
    serial_id_(serial_id + "_" + std::to_string(getpid())), 
    registered_station_(""), 
    registered_(false),
    emergency_(true),
    battery_(69),
    state_("MENTAL_CRISIS")
{
    geometry_msgs::msg::Point dummy;
    dummy.x = 69.0;
    dummy.y = 96.0;
    position_ = dummy;
    target_ = dummy;

    RCLCPP_INFO(this->get_logger(), "Robot Uplink node started with serial ID: '%s'", serial_id_.c_str());

    uplink_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Robot::RobotUplink::UplinkTimerCallback, this)
    );

    register_receiver_subscriber_ = this->create_subscription<unomas::msg::StrStr>(
        "Register", 10, std::bind(&Robot::RobotUplink::registerReceiverCallback, this, std::placeholders::_1));

    serial_broadcast_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "Serial", 10);

    register_serial_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "Registrar", 10);


    // emergency_publisher_ = this->create_publisher<std_msgs::msg::Bool>("dumpTopic00", 10);
    // battery_publisher_ = this->create_publisher<std_msgs::msg::Int32>("dumpTopic01", 10);
    // position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("dumpTopic02", 10);
    // state_publisher_ = this->create_publisher<std_msgs::msg::String>("dumpTopic03", 10);
    // target_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("dumpTopic04", 10);
}

Robot::RobotUplink::~RobotUplink()
{
    RCLCPP_INFO(this->get_logger(), "RobotUplink node for robot '%s' is shutting down.", serial_id_.c_str());
}

void Robot::RobotUplink::initialise()
{
    //controller_ = std::make_shared<RobotController>(serial_id_, shared_from_this());
}

void Robot::RobotUplink::goalCallback(const unomas::msg::AddressedPoseArray::SharedPtr msg)
{
    if (msg->address != serial_id_)
    {
        RCLCPP_WARN(this->get_logger(), "Received goals intended for '%s', but this robot's serial ID is '%s'. Ignoring.", msg->address.c_str(), serial_id_.c_str());
        return;
    }
    std::vector<geometry_msgs::msg::Point> goals;
    for (const auto& pose : msg->poses) {
        goals.push_back(pose.position);
    }
    //controller_->setGoals(goals);
    RCLCPP_INFO(this->get_logger(), "Received %zu goals for robot '%s'.", goals.size(), serial_id_.c_str());
    //controller_->autoNavigate();
}

void Robot::RobotUplink::UplinkTimerCallback()
{
    if(registered_station_.empty() && !registered_)
    {
        serial_broadcast_publisher_->publish(std_msgs::msg::String().set__data(serial_id_));
    } 
    else if(!registered_)
    {
        register_serial_publisher_ = this->create_publisher<std_msgs::msg::String>(
            registered_station_ + "/Registrar", 10);
        RCLCPP_INFO(this->get_logger(), "Registered with base station: '%s'", registered_station_.c_str());
        registered_ = true;
        register_serial_publisher_->publish(std_msgs::msg::String().set__data(serial_id_));
    }
    else
    {
        emergency_publisher_->publish(std_msgs::msg::Bool().set__data(emergency_));
        battery_publisher_->publish(std_msgs::msg::Int32().set__data(battery_));
        position_publisher_->publish(position_);
        state_publisher_->publish(std_msgs::msg::String().set__data(state_));
        target_publisher_->publish(target_);
    }
}

void Robot::RobotUplink::registerReceiverCallback(const unomas::msg::StrStr::SharedPtr msg)
{
    if(msg->textone == serial_id_ && registered_station_.empty())
    {
        registered_station_ = msg->texttwo;
        RCLCPP_INFO(this->get_logger(), "Received registration confirmation from base station: '%s'", registered_station_.c_str());
        
        goal_subscriber_ = this->create_subscription<unomas::msg::AddressedPoseArray>(
            registered_station_ + "/goals", 10, std::bind(&Robot::RobotUplink::goalCallback, this, std::placeholders::_1));

        std::string emergency_topic = registered_station_ + "/Status/Emergency";
        std::string battery_topic = registered_station_ + "/Status/Battery";
        std::string position_topic = registered_station_ + "/Status/Position";
        std::string state_topic = registered_station_ + "/Status/State";
        std::string target_topic = registered_station_ + "/Status/Target";

        emergency_publisher_ = this->create_publisher<std_msgs::msg::Bool>(emergency_topic, 10);
        battery_publisher_ = this->create_publisher<std_msgs::msg::Int32>(battery_topic, 10);
        position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(position_topic, 10);
        state_publisher_ = this->create_publisher<std_msgs::msg::String>(state_topic, 10);
        target_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(target_topic, 10);
    }
}