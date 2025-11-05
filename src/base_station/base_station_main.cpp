#include <rclcpp/rclcpp.hpp>
#include "base_station_status.h"
#include "base_station_register.h"
#include "base_station_terrain_soil.h"
#include "base_station_path.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto params = std::make_shared<rclcpp::Node>("params_base_station");

    params->declare_parameter<std::string>("station_name");
    std::string station_name;
    params->get_parameter("station_name", station_name);

    auto status_manager = std::make_shared<BaseStation::BaseStationStatusManager>(station_name);
    auto terrain_soil_manager = std::make_shared<BaseStation::BaseStationTerrainSoil>(station_name);
    auto registrar_manager = std::make_shared<BaseStation::BaseStationRegister>(station_name);

    registrar_manager->initialise();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(status_manager);
    executor.add_node(terrain_soil_manager);
    executor.add_node(registrar_manager);
    
    executor.spin();

    return 0;
}