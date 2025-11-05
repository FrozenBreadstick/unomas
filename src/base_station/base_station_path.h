#ifndef BASE_STATION_PATH_H
#define BASE_STATION_PATH_H

#include "rclcpp/rclcpp.hpp"
#include "unomas/msg/macro_plan.hpp"
#include "unomas/msg/addressed_pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <string>

namespace BaseStation
{
    class BaseStationPath
    {
        public:
            BaseStationPath(std::string station_name, const std::shared_ptr<rclcpp::Node>& node);
            ~BaseStationPath();

            void updateStoredRobots(std::vector<std::string> robots_);

        private:
            const std::shared_ptr<rclcpp::Node>& node_;
            std::string station_name_;

            std::vector<std::string> stored_robots_;

            rclcpp::Subscription<unomas::msg::MacroPlan>::SharedPtr macro_plan_subscriber_;
            rclcpp::Publisher<unomas::msg::AddressedPoseArray>::SharedPtr goals_publisher_;

            std::string macro_plan_topic_;
            std::string goals_topic_;

            void macroPlanCallback(const unomas::msg::MacroPlan::SharedPtr msg);

            unomas::msg::AddressedPoseArray computeShortestRoute(
                const unomas::msg::MacroPlan &plan,
                const geometry_msgs::msg::Point &start_position);
    };
}

#endif // BASE_STATION_PATH_H
