#include <rclcpp/rclcpp.hpp>
#include "simulation_extra.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto params = std::make_shared<rclcpp::Node>("params_simulation");

    params->declare_parameter<int>("size");
    params->declare_parameter<double>("resolution");
    int size;
    double resolution;
    params->get_parameter("size", size);
    params->get_parameter("resolution", resolution);

    auto simulator = std::make_shared<Simulation::SimulationExtra>(size, resolution);

    rclcpp::spin(simulator);

    return 0;
}