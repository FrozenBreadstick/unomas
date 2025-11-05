#ifndef SIMULATION_EXTRA_H
#define SIMULATION_EXTRA_H

#include "rclcpp/rclcpp.hpp"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/GridMap.hpp>

#include "unomas/srv/query_soil.hpp"
#include "unomas/msg/soil_info.hpp"

#include "unomas/srv/dummy_trigger.hpp"

#include <random>
#include <cmath>

namespace Simulation {
    class SimulationExtra : public rclcpp::Node
    {
        public:
            SimulationExtra(int size);
            ~SimulationExtra();

        private:
            void populateSoilMap(grid_map::GridMap& map);

            void soilQueryServiceCallback(const std::shared_ptr<unomas::srv::QuerySoil::Request> request,
                                      std::shared_ptr<unomas::srv::QuerySoil::Response> response);

            void soilDebugServiceCallback(const std::shared_ptr<unomas::srv::QuerySoil::Request> request,
                                      std::shared_ptr<unomas::srv::QuerySoil::Response> response);

            rclcpp::Service<unomas::srv::QuerySoil>::SharedPtr soil_query_service_;

            rclcpp::Service<unomas::srv::QuerySoil>::SharedPtr soil_debug_service_;
            rclcpp::Publisher<unomas::msg::SoilInfo>::SharedPtr soil_debug_publisher_;

            int size_;
            grid_map::GridMap soil_map_;
    };
}

#endif // SIMULATION_EXTRA_H