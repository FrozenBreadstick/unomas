#ifndef UI_BRIDGE_H
#define UI_BRIDGE_H

#include "rclcpp/rclcpp.hpp"

#include "unomas/srv/status_update_service.hpp"
#include "unomas/msg/status_update_packet.hpp"
#include "unomas/srv/terrain_soil_data.hpp"
#include "unomas/msg/soil_info.hpp"

namespace UI
{
    class UIBridge : public rclcpp::Node
    {
        public:
            UIBridge();
            ~UIBridge();

        private:
            void update(const std::shared_ptr<unomas::srv::StatusUpdateService::Request> /*request*/,
                                      std::shared_ptr<unomas::srv::StatusUpdateService::Response> response);
            rclcpp::Service<unomas::srv::StatusUpdateService>::SharedPtr status_update_service_;

            void packetReceiptSubCallback(const unomas::msg::StatusUpdatePacket::SharedPtr msg);
            rclcpp::Subscription<unomas::msg::StatusUpdatePacket>::SharedPtr packet_receipt_subscriber_;
            unomas::msg::StatusUpdatePacket latest_;
            std::mutex data_lock_;

            rclcpp::Service<unomas::srv::TerrainSoilData>::SharedPtr terrain_soil_service_;
            void terrainSoilServiceCallback(
                const std::shared_ptr<unomas::srv::TerrainSoilData::Request> request,
                std::shared_ptr<unomas::srv::TerrainSoilData::Response> response
            );

            void soilInfoSubCallback(const unomas::msg::SoilInfo::SharedPtr msg);
            rclcpp::Subscription<unomas::msg::SoilInfo>::SharedPtr soil_info_subscriber_;
            std::vector<unomas::msg::SoilInfo> soil_data_;
            std::mutex soil_data_lock_;
    };
}

#endif // UI_BRIDGE_H