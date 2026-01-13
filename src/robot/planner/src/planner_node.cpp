// planner_node.cpp
#include "planner_node.hpp"

#include <algorithm>
#include <unordered_set>

PlannerNode::PlannerNode()
: Node("planner"),
  planning_core_(robot::PlannerCore(this->get_logger())),
  state_(PlannerState::AWAITING_GOAL)
{
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::handleMapUpdate, this, std::placeholders::_1));

  goal_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::handleGoalUpdate, this, std::placeholders::_1));

  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::handleOdometryUpdate, this, std::placeholders::_1));

  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  planning_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1500), std::bind(&PlannerNode::onTimerEvent, this));
}

void PlannerNode::handleMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;

  if (goal_set_)
  {
    CellIndex goal_cell = mapToCell(goal_.point);

    if (goal_cell.x < 0 || goal_cell.x >= static_cast<int>(current_map_.info.width) ||
        goal_cell.y < 0 || goal_cell.y >= static_cast<int>(current_map_.info.height) ||
        cellCost(goal_cell) >= OBSTACLE_THRESHOLD)
    {
      state_ = PlannerState::AWAITING_GOAL;
      goal_set_ = false;
      publishEmptyPath();
      return;
    }
  }

  if (state_ == PlannerState::PLANNING_PATH)
  {
    if (isRobotInInflatedCell() && has_last_path_) return;
    computePath();
  }
}

void PlannerNode::handleGoalUpdate(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_set_ = true;
  state_ = PlannerState::PLANNING_PATH;

  if (isRobotInInflatedCell() && has_last_path_) return;
  computePath();
}

void PlannerNode::handleOdometryUpdate(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::onTimerEvent()
{
  if (state_ != PlannerState::PLANNING_PATH) return;

  if (hasGoalBeenReached())
  {
    state_ = PlannerState::AWAITING_GOAL;
    goal_set_ = false;
    publishEmptyPath();
    return;
  }

  if (isRobotInInflatedCell() && has_last_path_) return;

  computePath();
}

bool PlannerNode::hasGoalBeenReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.2;
}

bool PlannerNode::isRobotInInflatedCell()
{
  if (current_map_.data.empty()) return false;

  CellIndex start = mapToCell(robot_pose_.position);

  if (start.x < 0 || start.x >= static_cast<int>(current_map_.info.width) ||
      start.y < 0 || start.y >= static_cast<int>(current_map_.info.height))
  {
    return false;
  }

  double cost = cellCost(start);
  return (cost >= static_cast<double>(INFLATED_MIN_COST) && cost < static_cast<double>(OBSTACLE_THRESHOLD));
}

void PlannerNode::computePath()
{
  if (!goal_set_ || current_map_.data.empty()) return;

  if (isRobotInInflatedCell() && has_last_path_) return;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
  std::unordered_set<CellIndex, CellIndexHash> closed_list;

  node_registry_.clear();

  CellIndex start = mapToCell(robot_pose_.position);
  CellIndex target = mapToCell(goal_.point);

  if (start.x < 0 || start.x >= static_cast<int>(current_map_.info.width) ||
      start.y < 0 || start.y >= static_cast<int>(current_map_.info.height) ||
      target.x < 0 || target.x >= static_cast<int>(current_map_.info.width) ||
      target.y < 0 || target.y >= static_cast<int>(current_map_.info.height))
  {
    publishEmptyPath();
    return;
  }

  if (cellCost(start) >= OBSTACLE_THRESHOLD || cellCost(target) >= OBSTACLE_THRESHOLD)
  {
    publishEmptyPath();
    return;
  }

  double dx = static_cast<double>(start.x - target.x);
  double dy = static_cast<double>(start.y - target.y);
  double start_heuristic = std::sqrt(dx * dx + dy * dy);

  AStarNode start_node(start, start_heuristic, 0.0, start_heuristic, start);
  open_list.push(start_node);
  node_registry_[start] = start_node;

  while (!open_list.empty())
  {
    AStarNode current = open_list.top();
    open_list.pop();

    if (closed_list.count(current.index)) continue;

    if (current.index == target)
    {
      publishPath(retracePath(current));
      return;
    }

    closed_list.insert(current.index);

    for (const auto& neighbor : findNeighbors(current.index))
    {
      if (closed_list.count(neighbor)) continue;

      double cost_to_move = movementCost(current.index, neighbor);
      double tentative_g = current.g_score + cost_to_move;

      auto registry_entry = node_registry_.find(neighbor);

      if (registry_entry == node_registry_.end() || tentative_g < registry_entry->second.g_score)
      {
        double ndx = static_cast<double>(neighbor.x - target.x);
        double ndy = static_cast<double>(neighbor.y - target.y);
        double neighbor_heuristic = std::sqrt(ndx * ndx + ndy * ndy);

        AStarNode neighbor_node(neighbor, tentative_g + neighbor_heuristic, tentative_g, neighbor_heuristic, current.index);
        node_registry_[neighbor] = neighbor_node;
        open_list.push(neighbor_node);
      }
    }
  }

  publishEmptyPath();
}

std::vector<CellIndex> PlannerNode::findNeighbors(const CellIndex& current)
{
  static const std::vector<std::pair<int, int>> offsets = {
    {-1, -1}, {0, -1}, {1, -1},
    {-1,  0},          {1,  0},
    {-1,  1}, {0,  1}, {1,  1}
  };

  std::vector<CellIndex> neighbors;
  neighbors.reserve(8);

  for (const auto& offset : offsets)
  {
    int nx = current.x + offset.first;
    int ny = current.y + offset.second;

    if (nx >= 0 && nx < static_cast<int>(current_map_.info.width) &&
        ny >= 0 && ny < static_cast<int>(current_map_.info.height) &&
        cellCost(CellIndex(nx, ny)) < OBSTACLE_THRESHOLD)
    {
      neighbors.emplace_back(nx, ny);
    }
  }

  return neighbors;
}

double PlannerNode::cellCost(const CellIndex& cell)
{
  int index = cell.y * static_cast<int>(current_map_.info.width) + cell.x;
  return static_cast<double>(current_map_.data.at(static_cast<size_t>(index)));
}

CellIndex PlannerNode::mapToCell(const geometry_msgs::msg::Point& point)
{
  int x = static_cast<int>((point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int y = static_cast<int>((point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  return CellIndex(x, y);
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerNode::retracePath(const AStarNode& goal_node)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;

  AStarNode current = goal_node;

  while (!(current.index == current.parent))
  {
    path.insert(path.begin(), createPose(current.index));
    current = node_registry_[current.parent];
  }

  path.insert(path.begin(), createPose(current.index));
  return path;
}

geometry_msgs::msg::PoseStamped PlannerNode::createPose(const CellIndex& index)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = FRAME_ID;
  pose.header.stamp = this->get_clock()->now();
  pose.pose.position = cellToWorld(index);
  pose.pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Point PlannerNode::cellToWorld(const CellIndex& cell)
{
  geometry_msgs::msg::Point point;
  point.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
  point.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
  point.z = 0.0;
  return point;
}

void PlannerNode::publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = FRAME_ID;
  msg.poses = path;
  path_publisher_->publish(msg);

  last_path_ = path;
  has_last_path_ = true;
}

void PlannerNode::publishEmptyPath()
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = this->get_clock()->now();
  empty_path.header.frame_id = FRAME_ID;
  path_publisher_->publish(empty_path);

  last_path_.clear();
  has_last_path_ = false;
}

double PlannerNode::movementCost(const CellIndex& current, const CellIndex& neighbor)
{
  const double base = (std::abs(current.x - neighbor.x) == 1 && std::abs(current.y - neighbor.y) == 1) ? std::sqrt(2.0) : 1.0;
  const double cost = cellCost(neighbor);
  const double normalized = std::min(1.0, std::max(0.0, cost / static_cast<double>(OBSTACLE_THRESHOLD)));
  const double penalty = normalized * 0.5;
  return base + penalty;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

