#include "planner_core.hpp"

#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const
  {
    return x == other.x && y == other.y;
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex& idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode& a, const AStarNode& b)
  {
    return a.f_score > b.f_score;
  }
};

static inline bool inBounds(int x, int y, int w, int h)
{
  return x >= 0 && y >= 0 && x < w && y < h;
}

static inline int flatIndex(int x, int y, int w)
{
  return y * w + x;
}

static inline bool isBlocked(const nav_msgs::msg::OccupancyGrid& map, int x, int y)
{
  const int w = static_cast<int>(map.info.width);
  const int idx = flatIndex(x, y, w);
  const int8_t v = map.data[idx];
  if (v < 0) return true;
  return v >= 99;
}


static inline double heuristic(const CellIndex& a, const CellIndex& b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

static inline bool worldToCell(
  const nav_msgs::msg::OccupancyGrid& map,
  double wx, double wy,
  CellIndex& out)
{
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;

  const double gx = (wx - ox) / res;
  const double gy = (wy - oy) / res;

  const int cx = static_cast<int>(std::floor(gx));
  const int cy = static_cast<int>(std::floor(gy));

  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);

  if (!inBounds(cx, cy, w, h)) return false;

  out = CellIndex(cx, cy);
  return true;
}

static inline void cellToWorldCenter(
  const nav_msgs::msg::OccupancyGrid& map,
  const CellIndex& c,
  double& wx, double& wy)
{
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;

  wx = ox + (static_cast<double>(c.x) + 0.5) * res;
  wy = oy + (static_cast<double>(c.y) + 0.5) * res;
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger)
{
}

std::optional<nav_msgs::msg::Path> PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid& map,
  const nav_msgs::msg::Odometry& odom,
  const geometry_msgs::msg::PointStamped& goal)
{
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  if (w <= 0 || h <= 0) return std::nullopt;
  if (static_cast<int>(map.data.size()) != w * h) return std::nullopt;

  CellIndex start_cell;
  CellIndex goal_cell;

  const double sx = odom.pose.pose.position.x;
  const double sy = odom.pose.pose.position.y;

  const double gx = goal.point.x;
  const double gy = goal.point.y;

  if (!worldToCell(map, sx, sy, start_cell)) return std::nullopt;
  if (!worldToCell(map, gx, gy, goal_cell)) return std::nullopt;

  if (isBlocked(map, start_cell.x, start_cell.y)) return std::nullopt;
  if (isBlocked(map, goal_cell.x, goal_cell.y)) return std::nullopt;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start_cell] = 0.0;
  open.emplace(start_cell, heuristic(start_cell, goal_cell));

  const std::vector<CellIndex> dirs = {
    CellIndex(1, 0), CellIndex(-1, 0), CellIndex(0, 1), CellIndex(0, -1),
    CellIndex(1, 1), CellIndex(1, -1), CellIndex(-1, 1), CellIndex(-1, -1)
  };

  auto moveCost = [](const CellIndex& d) -> double {
    const int ax = std::abs(d.x);
    const int ay = std::abs(d.y);
    if (ax + ay == 1) return 1.0;
    return std::sqrt(2.0);
  };

  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  bool found = false;

  while (!open.empty())
  {
    const CellIndex current = open.top().index;
    open.pop();

    if (closed[current]) continue;
    closed[current] = true;

    if (current == goal_cell)
    {
      found = true;
      break;
    }

    const double current_g = g_score.count(current) ? g_score[current] : std::numeric_limits<double>::infinity();

    for (const auto& d : dirs)
    {
      const int nx = current.x + d.x;
      const int ny = current.y + d.y;

      if (!inBounds(nx, ny, w, h)) continue;
      if (isBlocked(map, nx, ny)) continue;

      const CellIndex neighbor(nx, ny);
      if (closed[neighbor]) continue;

      const double tentative_g = current_g + moveCost(d);

      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second)
      {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        const double f = tentative_g + heuristic(neighbor, goal_cell);
        open.emplace(neighbor, f);
      }
    }
  }

  if (!found) return std::nullopt;

  std::vector<CellIndex> cells;
  CellIndex cur = goal_cell;
  cells.push_back(cur);

  while (!(cur == start_cell))
  {
    auto it = came_from.find(cur);
    if (it == came_from.end()) break;
    cur = it->second;
    cells.push_back(cur);
  }

  if (!(cells.back() == start_cell)) return std::nullopt;

  std::reverse(cells.begin(), cells.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  path.poses.reserve(cells.size());
  for (const auto& c : cells)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";

    double wx, wy;
    cellToWorldCenter(map, c, wx, wy);

    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;

    path.poses.push_back(ps);
  }

  return path;
}

}