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
}

UI::UIBridge::~UIBridge()
{
    RCLCPP_INFO(this->get_logger(), "UI Bridge node is shutting down.");
}

void UI::UIBridge::packetReceiptSubCallback(const unomas::msg::StatusUpdatePacket::SharedPtr msg)
{
    latest_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received Status Update Packet from station: '%s'", latest_.name.c_str());
}

void UI::UIBridge::update(const std::shared_ptr<unomas::srv::StatusUpdateService::Request> /*request*/,
                              std::shared_ptr<unomas::srv::StatusUpdateService::Response> response)
{
    response->data = latest_;
    RCLCPP_INFO(this->get_logger(), "Sent Status Update Packet to UI for station: '%s'", latest_.name.c_str());
}