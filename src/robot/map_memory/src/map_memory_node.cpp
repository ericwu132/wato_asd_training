#include "map_memory_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath> //for sqrt, pow


using namespace std;
//publishers and subscribers
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) 
{
  RCLCPP_INFO(this->get_logger(), "Map Memory node started");

  //publish to a map topic
  map_memory_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  //subscribe to /costmap topic for nav_msg::msg::OccupancyGrid messages
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", rclcpp::SensorDataQoS(), 
  bind(&MapMemoryNode::topic_callback, this, _1)
  );
  //subscribe to /odom/filtered topic for nav_msg::msg::Odometry messages
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", rclcpp::SensorDataQoS(), 
  bind(&MapMemoryNode::topic_callback, this, _1)
  );

  //timer to limit update rate
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MapMemoryNode::timer_callback, this));
}

void MapMemoryNode::timer_callback()
{
  //call map memory core to get the current map
  nav_msgs::msg::OccupancyGrid map = map_memory_.get_map();

  //publish map
  map_memory_pub_->publish(map);

}

//why does chat and ros always call things callback

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;

  if (!initialized_)
  {
    initial_pose_x_ = current_x;
    initial_pose_y_ = current_y;
    last_pose_x_ = current_x;
    last_pose_y_ = current_y;
    initialized_ = true;
    
    RCLCPP_INFO(this->get_logger(), "initialized robot pos at (%.2f, %.2f)", initial_pose_x_, initial_pose_y_);

  }

  double dx = current_x - last_pose_x_;
  double dy = current_y - last_pose_y_;

  double distance = sqrt(pow(dx, 2) + pow(dy, 2))

  if (distance >= 1.5)
  {
    map_memory_.update_pose(dx, dy);
    last_pose_x_ = current_x;
    last_pose_y = current_y;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
