#include <chrono>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  
}

void MapMemoryNode::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map){
  if (global_map_.info.width == 0) {
    global_map_ = *map;
    global_map_.header.frame_id = "sim_world";
  }
  latest_costmap_ = *map;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom){
  current_x = odom->pose.pose.position.x;
  current_y = odom->pose.pose.position.y;
  tf2::Quaternion q(
    odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z,
    odom->pose.pose.orientation.w); 
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_theta);
  double distance = std::sqrt(std::pow((current_x-last_x),2) + std::pow((current_y-last_y), 2));
  if(distance >= distance_threshold){
    last_x = current_x;
    last_y = current_y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if(should_update_map_ && costmap_updated_){
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}

void MapMemoryNode::integrateCostmap() {
  for(int local_grid_y = 0; local_grid_y < dim; ++local_grid_y){
    for(int local_grid_x = 0; local_grid_x < dim; ++local_grid_x){
      if(latest_costmap_.data[local_grid_y*dim+ local_grid_x] != 0){
        double local_x = (local_grid_x - dim / 2)*res;
        double local_y = (local_grid_y - dim / 2)*res;
        double global_x = current_x + local_x * cos(current_theta) - local_y * sin(current_theta);
        double global_y = current_y + local_x * sin(current_theta) + local_y * cos(current_theta);
        int global_grid_x = int(std::floor(dim/2 + global_x / res));
        int global_grid_y = int(std::floor(dim/2 + global_y / res));
        global_map_.data[global_grid_y*dim + global_grid_x] = std::max(global_map_.data[global_grid_y*dim + global_grid_x], latest_costmap_.data[local_grid_y*dim + local_grid_x]);
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
