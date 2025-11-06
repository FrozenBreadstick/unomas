#include "base_station_path.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

namespace
{
    constexpr double kDefaultRouteWidth = 2.0;
    constexpr double kMergeTolerance = 0.6;

    inline bool isFinite(const geometry_msgs::msg::Point &p)
    {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    geometry_msgs::msg::Pose poseFromPoint(const geometry_msgs::msg::Point &pt)
    {
        geometry_msgs::msg::Pose pose;
        pose.position = pt;
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        return pose;
    }

    double distance2d(const geometry_msgs::msg::Point &a,
                      const geometry_msgs::msg::Point &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }
}

BaseStation::BaseStationPath::BaseStationPath(std::string station_name,
                                              const std::shared_ptr<rclcpp::Node> &node) :
    node_(node),
    station_name_(std::move(station_name)),
    macro_plan_topic_(station_name_ + "/macro_plan"),
    goals_topic_(station_name_ + "/goals")
{
    goals_publisher_ = node_->create_publisher<unomas::msg::AddressedPoseArray>(goals_topic_, 10);
    macro_plan_subscriber_ = node_->create_subscription<unomas::msg::MacroPlan>(
        macro_plan_topic_, 10,
        std::bind(&BaseStation::BaseStationPath::macroPlanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
                "BaseStationPath listening for macro plans on '%s'.",
                macro_plan_topic_.c_str());
}

BaseStation::BaseStationPath::~BaseStationPath()
{
    RCLCPP_INFO(node_->get_logger(), "BaseStation Pathing Node is Shutting Down");
}

void BaseStation::BaseStationPath::updateStoredRobots(std::vector<std::string> robots_)
{
    stored_robots_ = std::move(robots_);
}

void BaseStation::BaseStationPath::macroPlanCallback(const unomas::msg::MacroPlan::SharedPtr msg)
{
    if (!msg)
    {
        RCLCPP_WARN(node_->get_logger(), "Received null macro plan message.");
        return;
    }

    if (msg->station_name != station_name_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Plan addressed to station '%s' while this node manages '%s'. Ignoring.",
                    msg->station_name.c_str(), station_name_.c_str());
        return;
    }

    if (msg->robot_address.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Plan missing robot address; skipping.");
        return;
    }

    if (!stored_robots_.empty())
    {
        const bool known_robot = std::find(stored_robots_.begin(), stored_robots_.end(),
                                           msg->robot_address) != stored_robots_.end();
        if (!known_robot)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Plan references unknown robot '%s'. Known robots: %zu. Skipping.",
                        msg->robot_address.c_str(), stored_robots_.size());
            return;
        }
    }

    if (msg->routes.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Plan contained no routes.");
        return;
    }

    geometry_msgs::msg::Point start = msg->start;
    if (!isFinite(start))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Plan missing starting position; unable to compute route.");
        return;
    }

    const auto goals = computeShortestRoute(*msg, start);
    if (goals.poses.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "No viable route could be generated for robot '%s'.",
                    msg->robot_address.c_str());
        return;
    }

    goals_publisher_->publish(goals);
    RCLCPP_INFO(node_->get_logger(),
                "Published %zu goals for robot '%s'.",
                goals.poses.size(), goals.address.c_str());
}

unomas::msg::AddressedPoseArray BaseStation::BaseStationPath::computeShortestRoute(
    const unomas::msg::MacroPlan &plan,
    const geometry_msgs::msg::Point &start_position)
{
    struct Node
    {
        geometry_msgs::msg::Point point{};
        double width{kDefaultRouteWidth};
    };

    std::vector<Node> nodes;
    nodes.reserve(128);
    std::vector<std::vector<std::pair<std::size_t, double>>> edges;

    auto addNode = [&](const geometry_msgs::msg::Point &pt, double width) -> std::size_t {
        Node node;
        node.point = pt;
        node.width = width;
        nodes.push_back(node);
        edges.emplace_back();
        return nodes.size() - 1;
    };

    auto findOrAddNode = [&](const geometry_msgs::msg::Point &pt, double width) -> std::size_t {
        const double tolerance = std::max(kMergeTolerance, width * 0.5);
        for (std::size_t idx = 0; idx < nodes.size(); ++idx)
        {
            if (distance2d(pt, nodes[idx].point) <= tolerance)
            {
                nodes[idx].width = std::max(nodes[idx].width, width);
                return idx;
            }
        }
        return addNode(pt, width);
    };

    auto connect = [&](std::size_t a, std::size_t b) {
        if (a == b) return;
        const double cost = distance2d(nodes[a].point, nodes[b].point);
        if (cost <= 0.0) return;
        edges[a].push_back({b, cost});
        edges[b].push_back({a, cost});
    };

    for (const auto &route : plan.routes)
    {
        if (route.waypoints.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Route '%s' has fewer than 2 points; skipping.",
                        route.label.c_str());
            continue;
        }

        const double width = route.width > 0.0 ? route.width : kDefaultRouteWidth;
        std::size_t previous = findOrAddNode(route.waypoints.front(), width);
        for (std::size_t i = 1; i < route.waypoints.size(); ++i)
        {
            const std::size_t current = findOrAddNode(route.waypoints[i], width);
            connect(previous, current);
            previous = current;
        }
    }

    if (nodes.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "No usable route geometry available.");

        unomas::msg::AddressedPoseArray goals;

        return goals;
    }

    struct Target
    {
        std::size_t node_index;
        std::string label;
    };

    std::vector<Target> targets;
    targets.reserve(plan.fields.size());

    for (const auto &field : plan.fields)
    {
        if (!field.enabled) continue;
        if (!isFinite(field.gate))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Field '%s' gate has invalid coordinates; skipping.",
                        field.label.c_str());
            continue;
        }

        std::size_t best_idx = nodes.size();
        double best_dist = std::numeric_limits<double>::infinity();
        for (std::size_t idx = 0; idx < nodes.size(); ++idx)
        {
            const double d = distance2d(field.gate, nodes[idx].point);
            if (d < best_dist)
            {
                best_dist = d;
                best_idx = idx;
            }
        }

        if (best_idx == nodes.size()) continue;
        targets.push_back(Target{best_idx, field.label});
    }

    if (targets.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "No enabled fields could be matched to route waypoints.");

        unomas::msg::AddressedPoseArray goals;

        return goals;
    }

    const std::size_t route_node_count = nodes.size();
    const std::size_t start_idx = addNode(start_position, kDefaultRouteWidth);

    std::vector<std::pair<double, std::size_t>> nearest;
    nearest.reserve(route_node_count);
    for (std::size_t idx = 0; idx < route_node_count; ++idx)
    {
        nearest.emplace_back(distance2d(start_position, nodes[idx].point), idx);
    }
    std::sort(nearest.begin(), nearest.end(),
              [](const auto &lhs, const auto &rhs){ return lhs.first < rhs.first; });

    const std::size_t link_count = std::min<std::size_t>(3, nearest.size());
    for (std::size_t i = 0; i < link_count; ++i)
    {
        connect(start_idx, nearest[i].second);
    }

    if (edges[start_idx].empty())
    {
        connect(start_idx, nearest.front().second);
    }

    auto shortestPath = [&](std::size_t source, std::size_t target) -> std::vector<std::size_t> {
        const double inf = std::numeric_limits<double>::infinity();
        std::vector<double> dist(nodes.size(), inf);
        std::vector<std::size_t> previous(nodes.size(), nodes.size());

        using Item = std::pair<double, std::size_t>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        dist[source] = 0.0;
        previous[source] = source;
        queue.emplace(0.0, source);

        while (!queue.empty())
        {
            const auto [cost, idx] = queue.top();
            queue.pop();
            if (cost > dist[idx]) continue;
            if (idx == target) break;

            for (const auto &[neighbor, edge_cost] : edges[idx])
            {
                const double new_cost = cost + edge_cost;
                if (new_cost + 1e-6 < dist[neighbor])
                {
                    dist[neighbor] = new_cost;
                    previous[neighbor] = idx;
                    queue.emplace(new_cost, neighbor);
                }
            }
        }

        if (!std::isfinite(dist[target])) return {};

        std::vector<std::size_t> path;
        std::size_t current = target;
        while (current != source)
        {
            path.push_back(current);
            current = previous[current];
        }
        path.push_back(source);
        std::reverse(path.begin(), path.end());
        return path;
    };

    auto appendPath = [](std::vector<std::size_t> &trail,
                         const std::vector<std::size_t> &path)
    {
        if (path.empty()) return;
        if (trail.empty())
        {
            trail.insert(trail.end(), path.begin(), path.end());
        }
        else
        {
            trail.insert(trail.end(), path.begin() + 1, path.end());
        }
    };

    std::vector<std::size_t> final_nodes;
    std::size_t current_idx = start_idx;

    if (plan.visit_all_fields)
    {
        for (const auto &target : targets)
        {
            const auto path = shortestPath(current_idx, target.node_index);
            if (path.empty())
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Field '%s' waypoint is unreachable; skipping.",
                            target.label.c_str());
                continue;
            }
            appendPath(final_nodes, path);
            current_idx = target.node_index;
        }
    }
    else
    {
        std::vector<Target> remaining = targets;
        while (!remaining.empty())
        {
            std::vector<std::size_t> best_path;
            std::size_t best_index = 0;
            double best_cost = std::numeric_limits<double>::infinity();

            for (std::size_t i = 0; i < remaining.size(); ++i)
            {
                const auto path = shortestPath(current_idx, remaining[i].node_index);
                if (path.empty()) continue;
                double cost = 0.0;
                for (std::size_t j = 1; j < path.size(); ++j)
                {
                    cost += distance2d(nodes[path[j - 1]].point, nodes[path[j]].point);
                }
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_path = path;
                    best_index = i;
                }
            }

            if (best_path.empty())
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Remaining field waypoints are unreachable from current node.");
                break;
            }

            appendPath(final_nodes, best_path);
            current_idx = remaining[best_index].node_index;
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_index));
        }
    }

    if (final_nodes.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Generated path is empty.");
        
        unomas::msg::AddressedPoseArray goals;

        return goals;
    }

    if (!final_nodes.empty() && final_nodes.front() == start_idx)
    {
        final_nodes.erase(final_nodes.begin()); // drop the start node
    }
    final_nodes.erase(std::unique(final_nodes.begin(), final_nodes.end()), final_nodes.end());

    if (final_nodes.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Final path contained only the start position.");
        
        unomas::msg::AddressedPoseArray goals;

        return goals;
    }

    unomas::msg::AddressedPoseArray goals;
    goals.address = plan.robot_address;
    goals.poses.reserve(final_nodes.size());
    for (std::size_t idx : final_nodes)
    {
        goals.poses.push_back(poseFromPoint(nodes[idx].point));
    }
    return goals;
}
