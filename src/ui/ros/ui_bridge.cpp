#include "ui_bridge.h"

UI::UIBridge::UIBridge() : Node("ui_bridge_node")
{
    packet_receipt_subscriber_ = this->create_subscription<unomas::msg::StatusUpdatePacket>(
        "/unomas/UpdatePacket", 10, std::bind(&UI::UIBridge::packetReceiptSubCallback, this, std::placeholders::_1)
    );

    status_update_service_ = this->create_service<unomas::srv::StatusUpdateService>(
        "update",
        std::bind(&UI::UIBridge::update, this, std::placeholders::_1, std::placeholders::_2)
    );

    soil_info_subscriber_ = this->create_subscription<unomas::msg::SoilInfo>(
        "unomas/SoilUpdatePacket", 10, std::bind(&UI::UIBridge::soilInfoSubCallback, this, std::placeholders::_1)
    );

    terrain_soil_service_ = this->create_service<unomas::srv::TerrainSoilData>(
        "soil_update",
        std::bind(&UI::UIBridge::terrainSoilServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    debug_pose_service_ = this->create_service<unomas::srv::DummyTrigger>(
        "testPoseArray",
        std::bind(&UI::UIBridge::debugPoseServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

UI::UIBridge::~UIBridge()
{
    RCLCPP_INFO(this->get_logger(), "UI Bridge node is shutting down.");
}

void UI::UIBridge::packetReceiptSubCallback(const unomas::msg::StatusUpdatePacket::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_lock_);
    latest_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received Status Update Packet from station: '%s'", latest_.name.c_str());
}

void UI::UIBridge::update(const std::shared_ptr<unomas::srv::StatusUpdateService::Request> /*request*/,
                              std::shared_ptr<unomas::srv::StatusUpdateService::Response> response)
{
    std::lock_guard<std::mutex> lock(data_lock_);
    response->data = latest_;
    RCLCPP_INFO(this->get_logger(), "Sent Status Update Packet to UI for station: '%s'", latest_.name.c_str());
}

void UI::UIBridge::terrainSoilServiceCallback(
    const std::shared_ptr<unomas::srv::TerrainSoilData::Request> request,
    std::shared_ptr<unomas::srv::TerrainSoilData::Response> response
)
{
    std::lock_guard<std::mutex> lock(soil_data_lock_);
    auto req = request; //Not used but so colcon doesnt give me a yellow square which makes me sad

    RCLCPP_INFO(this->get_logger(), "TerrainSoilData service called. Returning %zu soil data entries.", soil_data_.size());

    response->soil_data = soil_data_;
}

void UI::UIBridge::soilInfoSubCallback(const unomas::msg::SoilInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(soil_data_lock_);
    soil_data_.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Received soil info at (%.2f, %.2f).", msg->x, msg->y);
}

void UI::UIBridge::debugPoseServiceCallback(const std::shared_ptr<unomas::srv::DummyTrigger::Request> request,
            std::shared_ptr<unomas::srv::DummyTrigger::Response> response)
{
    std::string topic = request->message + "/goals";
    debug_pose_publisher_ = this->create_publisher<unomas::msg::AddressedPoseArray>(topic, 10);

    unomas::msg::AddressedPoseArray array;

    for (int i = 0; i < 3; ++i)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = static_cast<float>(std::rand() % 10);
      pose.position.y = static_cast<float>(std::rand() % 10);
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      array.poses.push_back(pose);
    }

    array.address = request->message2;

    debug_pose_publisher_->publish(array);
    
    response->message3 = "";
    response->message4 = "";
}