#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  enum class State
  {
    WAITING_FOR_GOAL,
    ACTIVE
  };

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  bool atGoal() const;
  void planAndPublish();

  robot::PlannerCore planner_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
  rclcpp::TimerBase::SharedPtr planningTimer;

  nav_msgs::msg::OccupancyGrid latest_map_;
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::PointStamped latest_goal_;

  bool map_received_;
  bool odom_received_;
  bool goal_received_;

  bool map_updated_;
  bool goal_updated_;

  State state_;

  double goal_tolerance_m_;
  double replan_distance_m_;

  bool have_last_plan_start_;
  double last_plan_start_x_;
  double last_plan_start_y_;
};

#endif
