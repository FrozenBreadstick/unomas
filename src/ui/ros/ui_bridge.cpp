#include "ui_bridge.h"

#include <algorithm>

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

    macro_plan_service_ = this->create_service<unomas::srv::UpdateMacroPlan>(
        "macro_plan_update",
        std::bind(&UI::UIBridge::macroPlanServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
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

rclcpp::Publisher<unomas::msg::MacroPlan>::SharedPtr UI::UIBridge::ensurePlanPublisher(const std::string& topic)
{
    auto it = macro_plan_publishers_.find(topic);
    if (it != macro_plan_publishers_.end())
    {
        return it->second;
    }

    auto pub = this->create_publisher<unomas::msg::MacroPlan>(topic, 10);
    macro_plan_publishers_[topic] = pub;
    RCLCPP_INFO(this->get_logger(), "Created macro plan publisher on topic '%s'.", topic.c_str());
    return pub;
}

void UI::UIBridge::macroPlanServiceCallback(
    const std::shared_ptr<unomas::srv::UpdateMacroPlan::Request> request,
    std::shared_ptr<unomas::srv::UpdateMacroPlan::Response> response)
{
    if (!request)
    {
        response->accepted = false;
        response->message = "Empty request";
        return;
    }

    const auto& plan = request->plan;
    if (plan.station_name.empty())
    {
        response->accepted = false;
        response->message = "Plan must specify station_name.";
        return;
    }
    if (plan.robot_address.empty())
    {
        response->accepted = false;
        response->message = "Plan must specify robot_address.";
        return;
    }

    if (plan.routes.empty())
    {
        response->accepted = false;
        response->message = "Plan must contain at least one route corridor.";
        return;
    }

    const bool hasEnabledGate = std::any_of(
        plan.fields.begin(), plan.fields.end(),
        [](const auto &field){ return field.enabled; });

    if (!hasEnabledGate)
    {
        response->accepted = false;
        response->message = "No active field gates were supplied.";
        return;
    }

    const std::string topic = plan.station_name + "/macro_plan";

    {
        std::lock_guard<std::mutex> lock(macro_plan_lock_);
        auto publisher = ensurePlanPublisher(topic);
        publisher->publish(plan);
    }

    response->accepted = true;
    response->message = "Forwarded macro plan to " + topic;
    const std::size_t active_gates = std::count_if(plan.fields.begin(), plan.fields.end(),
                                                   [](const auto &f){return f.enabled;});
    RCLCPP_INFO(this->get_logger(), "Macro plan forwarded for station '%s' targeting '%s' with %zu routes and %zu gates.",
                plan.station_name.c_str(), plan.robot_address.c_str(), plan.routes.size(), active_gates);
}
