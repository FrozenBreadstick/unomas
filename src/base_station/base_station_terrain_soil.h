#ifndef BASE_STATION_TERRAIN_SOIL_H
#define BASE_STATION_TERRAIN_SOIL_H

#include "rclcpp/rclcpp.hpp"
#include "unomas/msg/soil_info.hpp"
#include "unomas/srv/terrain_soil_data.hpp"

namespace BaseStation
{
    class BaseStationTerrainSoil : public rclcpp::Node
    {
        public:
            BaseStationTerrainSoil(std::string station_name);
            ~BaseStationTerrainSoil();
        
        private:
            std::string station_name_;

            std::vector<unomas::msg::SoilInfo> soil_data_storage_;

            std::mutex resource_lock_;

            rclcpp::Subscription<unomas::msg::SoilInfo>::SharedPtr soil_info_subscriber_;
            void soilInfoSubCallback(const unomas::msg::SoilInfo::SharedPtr msg);

            rclcpp::Service<unomas::srv::TerrainSoilData>::SharedPtr terrain_soil_service_;
            void terrainSoilServiceCallback(
                const std::shared_ptr<unomas::srv::TerrainSoilData::Request> request,
                std::shared_ptr<unomas::srv::TerrainSoilData::Response> response
            );
    };
}

#endif // BASE_STATION_TERRAIN_SOIL_H