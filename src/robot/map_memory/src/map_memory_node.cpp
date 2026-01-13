#include "map_memory_node.hpp"

#include <cmath>
#include <functional>

using std::placeholders::_1;

static double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  core_(robot::MapMemoryCore(this->get_logger()))
{
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::costmapCallback, this, _1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::odomCallback, this, _1)
  );

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::timerCallback, this)
  );
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  have_costmap_ = true;

  if (!core_.isInitialized()) {
    core_.initializeFromCostmap(latest_costmap_);
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  cur_x_ = msg->pose.pose.position.x;
  cur_y_ = msg->pose.pose.position.y;

  const auto & q = msg->pose.pose.orientation;
  cur_yaw_ = yawFromQuaternion(q.x, q.y, q.z, q.w);

  have_odom_ = true;

  if (!initialized_motion_) {
    last_update_x_ = cur_x_;
    last_update_y_ = cur_y_;
    initialized_motion_ = true;
    moved_enough_ = false;
    return;
  }

  const double dx = cur_x_ - last_update_x_;
  const double dy = cur_y_ - last_update_y_;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist >= 1.5) {
    moved_enough_ = true;
  }
}

void MapMemoryNode::timerCallback()
{
  if (have_odom_ && have_costmap_ && moved_enough_ && core_.isInitialized()) {
    core_.integrateCostmap(latest_costmap_, cur_x_, cur_y_, cur_yaw_);
    last_update_x_ = cur_x_;
    last_update_y_ = cur_y_;
    moved_enough_ = false;
  }

  if (core_.isInitialized()) {
    auto msg = core_.getMap();
    msg.header.stamp = this->now();
    msg.header.frame_id = "sim_world";
    map_pub_->publish(msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
