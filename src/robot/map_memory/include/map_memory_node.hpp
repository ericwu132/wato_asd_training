#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void timerCallback();

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool have_costmap_ = false;

  double cur_x_ = 0.0;
  double cur_y_ = 0.0;
  double cur_yaw_ = 0.0;
  bool have_odom_ = false;

  double last_update_x_ = 0.0;
  double last_update_y_ = 0.0;
  bool initialized_motion_ = false;
  bool moved_enough_ = false;

  robot::MapMemoryCore core_;
};

#endif
