#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  // Callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void timerCallback();

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_memory_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Core logic
  robot::MapMemoryCore map_memory_;
  // State for distance-based updates
  double initial_pose_x_ = 0.0;
  double initial_pose_y_ = 0.0;
  double last_pose_x_ = 0.0;
  double last_pose_y_ = 0.0;
  bool initialized_ = false;
};

#endif
