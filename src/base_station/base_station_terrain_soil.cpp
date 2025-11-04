#include "base_station_terrain_soil.h"

BaseStation::BaseStationTerrainSoil::BaseStationTerrainSoil(std::string station_name): 
    Node("base_station_terrain_soil_node"), 
    station_name_(station_name)
{
    unomas::msg::SoilInfo sample_soil;
    sample_soil.x = 0.0;
    sample_soil.y = 0.0;
    sample_soil.moisture = 0.0;
    sample_soil.ph = 0.0;
    sample_soil.nutrients = 0.0;
    soil_data_storage_.push_back(sample_soil);

    soil_info_subscriber_ = this->create_subscription<unomas::msg::SoilInfo>(
        "unomas/soil",
        10,
        std::bind(&BaseStation::BaseStationTerrainSoil::soilInfoSubCallback, this, std::placeholders::_1)
    );

    terrain_soil_service_ = this->create_service<unomas::srv::TerrainSoilData>(
        "terrain_soil_data",
        std::bind(&BaseStation::BaseStationTerrainSoil::terrainSoilServiceCallback, this, std::placeholders::_1, std::placeholders::_2
        )
    );

}

BaseStation::BaseStationTerrainSoil::~BaseStationTerrainSoil()
{
    RCLCPP_INFO(this->get_logger(), "BaseStationTerrainSoil node shutting down.");
}

void BaseStation::BaseStationTerrainSoil::soilInfoSubCallback(const unomas::msg::SoilInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(resource_lock_);

    RCLCPP_INFO(this->get_logger(), "Received soil info at (%.2f, %.2f): Moisture=%.2f, pH=%.2f, Nutrients=%.2f",
        msg->x, msg->y, msg->moisture, msg->ph, msg->nutrients);


    for (const auto& stored_soil : soil_data_storage_)
    {
        if (stored_soil.x == msg->x && stored_soil.y == msg->y)
        {
            RCLCPP_INFO(this->get_logger(), "Soil data at (%.2f, %.2f) already exists. Skipping storage.", msg->x, msg->y);
            return;
        }
    }

    soil_data_storage_.push_back(*msg);
}

void BaseStation::BaseStationTerrainSoil::terrainSoilServiceCallback(
    const std::shared_ptr<unomas::srv::TerrainSoilData::Request> request,
    std::shared_ptr<unomas::srv::TerrainSoilData::Response> response
)
{
    std::lock_guard<std::mutex> lock(resource_lock_);

    auto req = request; //Not used but so colcon doesnt give me a yellow square which makes me sad

    RCLCPP_INFO(this->get_logger(), "TerrainSoilData service called. Returning %zu soil data entries.", soil_data_storage_.size());

    response->soil_data = soil_data_storage_;
}