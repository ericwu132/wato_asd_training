#include "control_core.hpp"

#include <cmath>
#include <limits>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger),
  lookahead_distance_(1.0),
  goal_tolerance_(0.15),
  linear_speed_(0.5),
  max_angular_speed_(1.5),
  have_path_(false),
  have_odom_(false)
{
}

void ControlCore::setLookahead(double meters)
{
  if (meters > 0.01) lookahead_distance_ = meters;
}

void ControlCore::setGoalTolerance(double meters)
{
  if (meters > 0.0) goal_tolerance_ = meters;
}

void ControlCore::setLinearSpeed(double mps)
{
  if (mps >= 0.0) linear_speed_ = mps;
}

void ControlCore::setMaxAngularSpeed(double rps)
{
  if (rps > 0.0) max_angular_speed_ = rps;
}

void ControlCore::updatePath(const nav_msgs::msg::Path& path)
{
  poses_.clear();
  poses_.reserve(path.poses.size());
  for (const auto& p : path.poses) poses_.push_back(p);

  frame_id_ = path.header.frame_id;
  have_path_ = !poses_.empty();
}

void ControlCore::updateOdom(const nav_msgs::msg::Odometry& odom)
{
  odom_ = odom;
  have_odom_ = true;
}

bool ControlCore::hasPath() const
{
  return have_path_;
}

bool ControlCore::hasOdom() const
{
  return have_odom_;
}

double ControlCore::normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double ControlCore::distance2D(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::yawFromQuat(double x, double y, double z, double w)
{
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

std::optional<size_t> ControlCore::findNearestIndex(double rx, double ry) const
{
  if (poses_.empty()) return std::nullopt;

  double best = std::numeric_limits<double>::infinity();
  size_t best_i = 0;

  for (size_t i = 0; i < poses_.size(); ++i)
  {
    const double px = poses_[i].pose.position.x;
    const double py = poses_[i].pose.position.y;
    const double d = (px - rx) * (px - rx) + (py - ry) * (py - ry);
    if (d < best)
    {
      best = d;
      best_i = i;
    }
  }

  return best_i;
}

std::optional<size_t> ControlCore::findLookaheadIndex(double rx, double ry, size_t start_idx) const
{
  (void)rx;
  (void)ry;

  if (poses_.empty()) return std::nullopt;
  if (start_idx >= poses_.size()) start_idx = poses_.size() - 1;
  if (poses_.size() == 1) return 0;

  double accum = 0.0;

  size_t i = start_idx;
  double prev_x = poses_[i].pose.position.x;
  double prev_y = poses_[i].pose.position.y;

  for (size_t k = i + 1; k < poses_.size(); ++k)
  {
    const double cx = poses_[k].pose.position.x;
    const double cy = poses_[k].pose.position.y;

    accum += distance2D(prev_x, prev_y, cx, cy);

    if (accum >= lookahead_distance_) return k;

    prev_x = cx;
    prev_y = cy;
  }

  return poses_.size() - 1;
}

std::optional<geometry_msgs::msg::Twist> ControlCore::step()
{
  if (!have_path_ || !have_odom_ || poses_.empty()) return std::nullopt;

  const double rx = odom_.pose.pose.position.x;
  const double ry = odom_.pose.pose.position.y;

  const auto& q = odom_.pose.pose.orientation;
  const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);

  const double gx = poses_.back().pose.position.x;
  const double gy = poses_.back().pose.position.y;

  const double dist_to_goal = distance2D(rx, ry, gx, gy);
  if (dist_to_goal <= goal_tolerance_)
  {
    geometry_msgs::msg::Twist stop;
    return stop;
  }

  auto nearest_opt = findNearestIndex(rx, ry);
  if (!nearest_opt.has_value()) return std::nullopt;
  const size_t nearest = nearest_opt.value();

  auto lookahead_opt = findLookaheadIndex(rx, ry, nearest);
  if (!lookahead_opt.has_value()) return std::nullopt;
  const size_t look_i = lookahead_opt.value();

  const double tx = poses_[look_i].pose.position.x;
  const double ty = poses_[look_i].pose.position.y;

  const double dx = tx - rx;
  const double dy = ty - ry;

  const double target_heading = std::atan2(dy, dx);
  const double alpha = normalizeAngle(target_heading - yaw);

  const double L = std::max(lookahead_distance_, 0.01);
  const double curvature = (2.0 * std::sin(alpha)) / L;

  double v = linear_speed_;

  const double heading_scale = std::max(0.0, std::cos(alpha));
  v *= heading_scale;

  if (dist_to_goal < 1.5 * lookahead_distance_)
  {
    const double scale = std::max(0.1, dist_to_goal / (1.5 * lookahead_distance_));
    v *= scale;
  }

  double w;

  if (std::abs(alpha) > 1.2)
  {
    v = 0.0;
    w = (alpha > 0.0) ? max_angular_speed_ : -max_angular_speed_;
  }
  else
  {
    w = v * curvature;
    if (w > max_angular_speed_) w = max_angular_speed_;
    if (w < -max_angular_speed_) w = -max_angular_speed_;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = w;
  return cmd;
}

}
