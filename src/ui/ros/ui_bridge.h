#ifndef UI_BRIDGE_H
#define UI_BRIDGE_H

#include "rclcpp/rclcpp.hpp"

#include "unomas/srv/status_update_service.hpp"
#include "unomas/msg/status_update_packet.hpp"

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
    };
}

#endif // UI_BRIDGE_H