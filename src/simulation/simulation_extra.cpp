#include "simulation_extra.h"

Simulation::SimulationExtra::SimulationExtra(int size)
    : Node("simulation_extras_node"), size_(size)
{
    soil_map_.setFrameId("map");
    soil_map_.setGeometry(grid_map::Length(size_, size_), 1);
    soil_map_.add("moisture");
    soil_map_.add("ph");
    soil_map_.add("nutrients");

    populateSoilMap(soil_map_);

    soil_query_service_ = this->create_service<unomas::srv::QuerySoil>(
        "query_soil",
        std::bind(&SimulationExtra::soilQueryServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    soil_debug_service_ = this->create_service<unomas::srv::QuerySoil>(
        "query_soil",
        std::bind(&SimulationExtra::soilDebugServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    soil_debug_publisher_ = this->create_publisher<unomas::msg::SoilInfo>("unomas/SoilUpdatePacket", 10);
}

Simulation::SimulationExtra::~SimulationExtra()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down SimulationExtra node.");
}

/**
 * @brief Randomly populate all grid map layers between 0 and 1
 */
void Simulation::SimulationExtra::populateSoilMap(grid_map::GridMap& map)
{
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_real_distribution<double> dist(0,1.0);

    for (const auto& layer : map.getLayers()) { //All layers
        RCLCPP_INFO(this->get_logger(), "Populating simulation layer: %s", layer.c_str());
        auto& data = map[layer]; //DO IT FAST (access the matrix directly to prevent continuous bounds checks used by the grid map iterator)
        for (int i = 0; i < data.rows(); ++i)
        for (int j = 0; j < data.cols(); ++j)
            data(i, j) = dist(gen);
    }
}

void Simulation::SimulationExtra::soilQueryServiceCallback(
    const std::shared_ptr<unomas::srv::QuerySoil::Request> request,
    std::shared_ptr<unomas::srv::QuerySoil::Response> response)
{
    double x = request->x;
    double y = request->y;

    grid_map::Position pos(x, y);

    if (soil_map_.isInside(pos)) {
        response->moisture = soil_map_.atPosition("moisture", pos);
        response->ph       = soil_map_.atPosition("ph", pos);
        response->nutrients= soil_map_.atPosition("nutrients", pos);
    } else {
        response->moisture = response->ph = response->nutrients = NAN;
        RCLCPP_WARN(this->get_logger(), "Query position (%.2f, %.2f) is outside the soil map!", x, y);
    }
}

void Simulation::SimulationExtra::soilDebugServiceCallback(
    const std::shared_ptr<unomas::srv::QuerySoil::Request> request,
    std::shared_ptr<unomas::srv::QuerySoil::Response> response)
{
    double x = request->x;
    double y = request->y;

    unomas::msg::SoilInfo debugInfo;
    debugInfo.x = x;
    debugInfo.y = y;

    grid_map::Position pos(x, y);

    if (soil_map_.isInside(pos)) {
        response->moisture = soil_map_.atPosition("moisture", pos);
        response->ph       = soil_map_.atPosition("ph", pos);
        response->nutrients= soil_map_.atPosition("nutrients", pos);
    } else {
        response->moisture = response->ph = response->nutrients = NAN;
        RCLCPP_WARN(this->get_logger(), "Query position (%.2f, %.2f) is outside the soil map!", x, y);
    }

    debugInfo.moisture = response->moisture;
    debugInfo.ph = response->ph;
    debugInfo.nutrients = response->nutrients;
    soil_debug_publisher_->publish(debugInfo);
}