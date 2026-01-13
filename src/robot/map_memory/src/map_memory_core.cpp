#include "map_memory_core.hpp"

#include <cmath>
#include <algorithm>

namespace robot
{

static constexpr double GLOBAL_RESOLUTION = 0.1;
static constexpr int GLOBAL_SIZE = 500;

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger)
{
}

bool MapMemoryCore::isInitialized() const
{
  return initialized_;
}

void MapMemoryCore::initializeFromCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  (void)costmap;

  global_.info.resolution = GLOBAL_RESOLUTION;
  global_.info.width = GLOBAL_SIZE;
  global_.info.height = GLOBAL_SIZE;

  const double half = -static_cast<double>(GLOBAL_SIZE) * GLOBAL_RESOLUTION / 2.0;
  global_.info.origin.position.x = half;
  global_.info.origin.position.y = half;
  global_.info.origin.position.z = 0.0;
  global_.info.origin.orientation.x = 0.0;
  global_.info.origin.orientation.y = 0.0;
  global_.info.origin.orientation.z = 0.0;
  global_.info.origin.orientation.w = 1.0;

  global_.header.frame_id = "sim_world";
  global_.data.assign(static_cast<size_t>(GLOBAL_SIZE) * static_cast<size_t>(GLOBAL_SIZE), 0);

  initialized_ = true;
}

bool MapMemoryCore::inBounds(int gx, int gy) const
{
  return gx >= 0 && gy >= 0 &&
         gx < static_cast<int>(global_.info.width) &&
         gy < static_cast<int>(global_.info.height);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw)
{
  if (!initialized_) {
    return;
  }

  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);

  const int w = static_cast<int>(costmap.info.width);
  const int h = static_cast<int>(costmap.info.height);
  const double lres = costmap.info.resolution;
  const double lox = costmap.info.origin.position.x;
  const double loy = costmap.info.origin.position.y;

  const double gres = global_.info.resolution;
  const double gox = global_.info.origin.position.x;
  const double goy = global_.info.origin.position.y;

  for (int iy = 0; iy < h; ++iy) {
    const double ly = loy + (static_cast<double>(iy) + 0.5) * lres;
    const double sin_ly = s * ly;
    const double cos_ly = c * ly;

    for (int ix = 0; ix < w; ++ix) {
      const int idx = iy * w + ix;
      const int8_t v = costmap.data[idx];
      if (v < 0) {
        continue;
      }

      const double lx = lox + (static_cast<double>(ix) + 0.5) * lres;

      const double gx_world = robot_x + (c * lx - sin_ly);
      const double gy_world = robot_y + (s * lx + cos_ly);

      const int gx = static_cast<int>(std::floor((gx_world - gox) / gres));
      const int gy = static_cast<int>(std::floor((gy_world - goy) / gres));

      if (!inBounds(gx, gy)) {
        continue;
      }

      const int gidx = gy * static_cast<int>(global_.info.width) + gx;

      int8_t & oldv = global_.data[gidx];
      const int8_t old_nonneg = (oldv < 0) ? 0 : oldv;
      oldv = std::max(old_nonneg, v);
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap() const
{
  return global_;
}

}
