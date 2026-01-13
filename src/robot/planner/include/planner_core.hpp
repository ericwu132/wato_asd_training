#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot
{

class PlannerCore
{
public:
  explicit PlannerCore(const rclcpp::Logger& logger);

  std::optional<nav_msgs::msg::Path> plan(
    const nav_msgs::msg::OccupancyGrid& map,
    const nav_msgs::msg::Odometry& odom,
    const geometry_msgs::msg::PointStamped& goal);

private:
  rclcpp::Logger logger_;
};

}

#endif
