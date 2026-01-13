#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) 
{
  map_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  goal_point_sub_ = this->create_subscription<nav_msgs::msg::PointStamped>(
    "/goal_point",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::goalpointCallback, this, _1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::odomCallback, this, _1)
  );

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/odom/filtered",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::odomCallback, this, _1)
  );

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::timerCallback, this)
  );
}
}

void PlannerNode::goalpointCallback(const nav_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_x_ = msg->point.x;
  goal_y_ = msg->point.y;
  have_goal_ = true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
