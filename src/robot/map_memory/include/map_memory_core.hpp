#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  bool isInitialized() const;

  void initializeFromCostmap(const nav_msgs::msg::OccupancyGrid& costmap);

  void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                        double robot_x, double robot_y, double robot_yaw);

  nav_msgs::msg::OccupancyGrid getMap() const;

private:
  bool inBounds(int gx, int gy) const;

  rclcpp::Logger logger_;
  bool initialized_ = false;
  nav_msgs::msg::OccupancyGrid global_;
};

}

#endif
