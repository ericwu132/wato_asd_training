#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    robot::MapMemoryCore map_memory_;
    double initial_pose_x_ = 0;
    double initial_pose_y_ = 0;
    double last_pose_x_ = 0;
    double last_pose_y_ = 0;
    bool initialized_ = false;
};

#endif 
