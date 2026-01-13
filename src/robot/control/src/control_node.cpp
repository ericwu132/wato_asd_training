#include "control_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

ControlNode::ControlNode()
: Node("control"),
  core_(robot::ControlCore(this->get_logger()))
{
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("goal_tolerance", 0.15);
  this->declare_parameter<double>("linear_speed", 1.5);
  this->declare_parameter<double>("max_angular_speed", 3);
  this->declare_parameter<int>("control_hz", 10);

  const double lookahead = this->get_parameter("lookahead_distance").as_double();
  const double goal_tol = this->get_parameter("goal_tolerance").as_double();
  const double lin = this->get_parameter("linear_speed").as_double();
  const double max_w = this->get_parameter("max_angular_speed").as_double();
  const int hz = this->get_parameter("control_hz").as_int();

  core_.setLookahead(lookahead);
  core_.setGoalTolerance(goal_tol);
  core_.setLinearSpeed(lin);
  core_.setMaxAngularSpeed(max_w);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  const int period_ms = (hz <= 0) ? 100 : static_cast<int>(1000 / hz);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  core_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  core_.updateOdom(*msg);
}

void ControlNode::timerCallback()
{
  if (!core_.hasOdom() || !core_.hasPath()) return;

  auto cmd_opt = core_.step();
  if (!cmd_opt.has_value()) return;

  cmd_pub_->publish(cmd_opt.value());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
