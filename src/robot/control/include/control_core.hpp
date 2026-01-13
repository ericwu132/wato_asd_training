#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore
{
public:
  explicit ControlCore(const rclcpp::Logger& logger);

  void setLookahead(double meters);
  void setGoalTolerance(double meters);
  void setLinearSpeed(double mps);
  void setMaxAngularSpeed(double rps);

  void updatePath(const nav_msgs::msg::Path& path);
  void updateOdom(const nav_msgs::msg::Odometry& odom);

  bool hasPath() const;
  bool hasOdom() const;

  std::optional<geometry_msgs::msg::Twist> step();

private:
  static double normalizeAngle(double a);
  static double distance2D(double ax, double ay, double bx, double by);
  static double yawFromQuat(double x, double y, double z, double w);

  std::optional<size_t> findNearestIndex(double rx, double ry) const;
  std::optional<size_t> findLookaheadIndex(double rx, double ry, size_t start_idx) const;

  rclcpp::Logger logger_;

  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;
  double max_angular_speed_;

  bool have_path_;
  bool have_odom_;

  std::string frame_id_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  nav_msgs::msg::Odometry odom_;
};

}

#endif
