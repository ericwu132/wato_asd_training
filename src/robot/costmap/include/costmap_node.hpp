#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  // Callback for incoming lidar scans
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Costmap core + storage
  robot::CostmapCore costmap_;
  std::vector<int8_t> grid_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

#endif