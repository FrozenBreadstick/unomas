#include <rclcpp/rclcpp.hpp>
#include "ui_bridge.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto bridge = std::make_shared<UI::UIBridge>();

    rclcpp::spin(bridge);

    return 0;
}