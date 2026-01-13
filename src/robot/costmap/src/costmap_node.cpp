#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_node.hpp"

using std::placeholders::_1;

using namespace std;

static double RESOLUTION = 0.1; // meters per cell
static int WIDTH  = 200;        // cells
static int HEIGHT = 200;        // cells


//obtain info
CostmapNode::CostmapNode():Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  RCLCPP_INFO(this->get_logger(), "Costmap node started");

  // ublish to foxglove
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  //subscribe to lidar information
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",rclcpp::SensorDataQoS(), 
  bind(&CostmapNode::topic_callback, this, _1)
  );

  //allocate grid once
  grid_.assign(WIDTH * HEIGHT, 0);
}

//takes in data from lidar to "inflate"
static void inflate_obstacles(vector<int8_t>& costmap, double inflation_radius_m, int8_t max_cost) //max cost for inflated cells
{
  const int radius_cells = static_cast<int>(ceil(inflation_radius_m / RESOLUTION)); //get amount of cells around robot

  if (radius_cells <= 0) 
  {
    return;
  }

  //copies costmap
  vector<int8_t> original = costmap;

  //loops thru entire grid, gets index for each cell
  for (int y = 0; y < HEIGHT; y++) { 
    for (int x = 0; x < WIDTH; x++) {
      const int idx = y * WIDTH + x;
      

      //if it ISN'T an obstacle, it just goes to the next cell (need to find an obstacle to inflate)
      if (original[idx] != 100) 
      {
        //otherwise continue
        continue;
      }  

      //bounds, makes sure you don't go outside of the valid coordinates
      const int xlower = max(0, x - radius_cells); //smallest x value
      const int xupper = min(WIDTH - 1, x + radius_cells); //largest x value
      const int ylower = max(0, y - radius_cells); //smallest y value
      const int yupper = min(HEIGHT - 1, y + radius_cells); //largest y value


      //now we have found a cell that is an obstacle, need to inflate around it

      
      for (int ny = ylower; ny <= yupper; ny++) { //starting in most bottom, while less than max top bound, increment
        for (int nx = xlower; nx <= xupper; nx++) { //starting in leftmost bound, while less than rightmost bound, increment
          const int nidx = ny * WIDTH + nx; //current index being looked at

          //want to keep obstacles as obstacles
          if (costmap[nidx] == 100)
          {
            continue;
          }
          //these variables calculate the distance between the guaranteed object and the cell being looked at
          const int dx = nx - x;
          const int dy = ny - y;

          //
          const double dist_m = sqrt(static_cast<double>(dx * dx + dy * dy)) * RESOLUTION;

          if (dist_m > inflation_radius_m)
          {
            continue;
          }

          // cost = max_cost * (1 - dist / radius)
          const double raw = static_cast<double>(max_cost) * (1.0 - (dist_m / inflation_radius_m));
          const int8_t cost = static_cast<int8_t>(round(max(0.0, raw)));

          // Only overwrite if higher
          if (cost > costmap[nidx]) {
            costmap[nidx] = cost;
          }
        }
      }
    }
  }
}

//runs every time a lidar scan is received
void CostmapNode::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 1) Clear grid each scan (basic version)
  fill(grid_.begin(), grid_.end(), 0);

  // Put robot at center of grid
  const int origin_x = WIDTH / 2;
  const int origin_y = HEIGHT / 2;

  // 2) LaserScan -> occupied cells
  for (size_t i = 0; i < msg->ranges.size(); i++) 
  {
    const float r = msg->ranges[i];
    if (!isfinite(r)) continue;
    if (r < msg->range_min || r > msg->range_max) continue;

    const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

    // Polar -> Cartesian (meters, in lidar/robot frame)
    const double x = static_cast<double>(r) * cos(angle);
    const double y = static_cast<double>(r) * sin(angle);

    // Cartesian -> grid indices
    const int gx = static_cast<int>(floor(x / RESOLUTION)) + origin_x;
    const int gy = static_cast<int>(floor(y / RESOLUTION)) + origin_y;

    if (gx < 0 || gx >= WIDTH || gy < 0 || gy >= HEIGHT) continue;

    const int idx = gy * WIDTH + gx;
    grid_[idx] = 100; // occupied
  }

  inflate_obstacles(grid_, 1.4, 80);


  // 3) Publish OccupancyGrid
  nav_msgs::msg::OccupancyGrid out;
  out.header.stamp = this->now();
  out.header.frame_id = "robot/chassis/lidar";  

  out.info.resolution = RESOLUTION;
  out.info.width = WIDTH;
  out.info.height = HEIGHT;

  // Origin is the world coordinate of cell (0,0).
  // Since robot is at center, origin is negative half-width/height in meters.
  out.info.origin.position.x = -(WIDTH  * RESOLUTION) / 2.0;
  out.info.origin.position.y = -(HEIGHT * RESOLUTION) / 2.0;
  out.info.origin.position.z = 0.0;


  out.info.origin.orientation.w = 1.0;

  out.data = grid_;
  costmap_pub_->publish(out);

  // 4) Debug at max 2 Hz
  int occupied = 0;
  for (auto v : grid_) if (v == 100) occupied++;

  int inflated = 0;
for (auto v : grid_) if (v > 0) inflated++;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}