#include "base_station_path.h"

BaseStation::BaseStationPath::BaseStationPath(std::string station_name, const std::shared_ptr<rclcpp::Node>& node) :
    node_(node),
    station_name_(station_name)
{

}

BaseStation::BaseStationPath::~BaseStationPath()
{
    RCLCPP_INFO(node_->get_logger(), "BaseStation Pathing Node is Shutting Down");
}