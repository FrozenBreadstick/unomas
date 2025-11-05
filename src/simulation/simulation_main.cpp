#include <rclcpp/rclcpp.hpp>
#include "simulation_extra.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto params = std::make_shared<rclcpp::Node>("params_simulation");

    params->declare_parameter<int>("size");
    int size;
    params->get_parameter("size", size);

    auto simulator = std::make_shared<Simulation::SimulationExtra>(size);

    rclcpp::spin(simulator);

    return 0;
}