#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  string_pub_->publish(message);
}
 
void CostmapNode::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr scan){

  const double res = 0.1;
  const int dim = 60/res;
  std::vector<std::vector<int>> occupancy_grid(dim, std::vector<int>(dim, 0));

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
        // Calculate grid coordinates
        double x = range*std::cos(angle), y = range*std::sin(angle);
        int x_grid = int(std::floor(dim/2 + x / res)), y_grid = int(std::floor(dim/2 + y / res));
        occupancy_grid[x_grid][y_grid] = 100;
    }
  } 

  const double inflation_radius = 1;
  int inflation_cells = std::ceil(inflation_radius / res);

  for(int x = 0; x < dim; ++x){
    for(int y = 0; y < dim; ++y){
      if(occupancy_grid[x][y] == 100){
        for(int dx = -inflation_cells; dx*res <= inflation_cells; ++dx){
          for(int dy = -inflation_cells; dy*res <= inflation_cells; ++dy){
            double distance = std::sqrt(std::pow(dx*res, 2) + std::pow(dy*res, 2));
            if(x+dx >= 0 && x+dx < dim && y+dy >= 0 && y+dy < dim && distance < inflation_radius){
              occupancy_grid[x+dx][y+dy] = std::max(occupancy_grid[x+dx][y+dy], int(100*(1-distance/inflation_radius)));
            }
          }
        }
      }
    }
  }

  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header = scan->header; // Set the frame_id
  msg.info.resolution = res;  // Each grid cell represents 5cm
  msg.info.width = dim;         // 10x10 grid
  msg.info.height = dim;
  msg.info.origin.position.x = -dim * res / 2;
  msg.info.origin.position.y = -dim * res / 2;
  msg.data.resize(msg.info.width * msg.info.height, -1);
  for(int y = 0; y < dim; ++y){
    for(int x = 0; x < dim; ++x){
      msg.data[y*dim + x] = occupancy_grid[x][y];
    }
  }

  costmap_pub_->publish(msg);
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
