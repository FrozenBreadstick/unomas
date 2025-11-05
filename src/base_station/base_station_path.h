#ifndef BASE_STATION_PATH_H
#define BASE_STATION_PATH_H

#include "rclcpp/rclcpp.hpp"

namespace BaseStation
{
    class BaseStationPath
    {
        public:
            BaseStationPath(std::string station_name, const std::shared_ptr<rclcpp::Node>& node);
            ~BaseStationPath();
        
        private:
            const std::shared_ptr<rclcpp::Node>& node_;
            std::string station_name_;

    };
}

#endif // BASE_STATION_PATH_H