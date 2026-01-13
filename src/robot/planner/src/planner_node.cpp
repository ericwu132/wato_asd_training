#include "planner_node.hpp"

#include <cmath>

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger())),
  map_received_(false),
  odom_received_(false),
  goal_received_(false),
  map_updated_(false),
  goal_updated_(false),
  state_(State::WAITING_FOR_GOAL),
  goal_tolerance_m_(0.25),
  replan_distance_m_(0.50),
  have_last_plan_start_(false),
  last_plan_start_x_(0.0),
  last_plan_start_y_(0.0)
{
  mapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goalSub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  planningTimer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = *msg;
  map_received_ = true;
  map_updated_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  latest_goal_ = *msg;
  goal_received_ = true;
  goal_updated_ = true;
  if (state_ == State::WAITING_FOR_GOAL) state_ = State::ACTIVE;
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  odom_received_ = true;
}

bool PlannerNode::atGoal() const
{
  if (!goal_received_ || !odom_received_) return false;

  const double dx = latest_goal_.point.x - latest_odom_.pose.pose.position.x;
  const double dy = latest_goal_.point.y - latest_odom_.pose.pose.position.y;
  const double d = std::sqrt(dx * dx + dy * dy);
  return d <= goal_tolerance_m_;
}

void PlannerNode::planAndPublish()
{
  if (!map_received_ || !odom_received_ || !goal_received_) return;

  auto planned = planner_.plan(latest_map_, latest_odom_, latest_goal_);
  if (!planned.has_value())
  {
    RCLCPP_WARN(this->get_logger(), "No valid path found");
    map_updated_ = false;
    goal_updated_ = false;
    return;
  }

  nav_msgs::msg::Path path = planned.value();
  path.header.stamp = this->now();
  for (auto& ps : path.poses) ps.header.stamp = path.header.stamp;

  pathPublisher->publish(path);

  last_plan_start_x_ = latest_odom_.pose.pose.position.x;
  last_plan_start_y_ = latest_odom_.pose.pose.position.y;
  have_last_plan_start_ = true;

  map_updated_ = false;
  goal_updated_ = false;
}

void PlannerNode::timerCallback()
{
  if (!map_received_ || !odom_received_) return;

  if (state_ == State::WAITING_FOR_GOAL)
  {
    if (!goal_received_) return;
    state_ = State::ACTIVE;
  }

  if (!goal_received_) return;

  if (atGoal())
  {
    state_ = State::WAITING_FOR_GOAL;
    goal_received_ = false;
    goal_updated_ = false;
    have_last_plan_start_ = false;
    return;
  }

  bool need_replan = false;

  if (goal_updated_ || map_updated_) need_replan = true;

  if (!need_replan)
  {
    if (!have_last_plan_start_)
    {
      need_replan = true;
    }
    else
    {
      const double cx = latest_odom_.pose.pose.position.x;
      const double cy = latest_odom_.pose.pose.position.y;
      const double dx = cx - last_plan_start_x_;
      const double dy = cy - last_plan_start_y_;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d >= replan_distance_m_) need_replan = true;
    }
  }

  if (need_replan) planAndPublish();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
