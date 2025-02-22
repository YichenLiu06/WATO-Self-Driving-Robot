#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"



class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void updateMap();
    void integrateCostmap();
    void costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom);


  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x = 0.0, last_y = 0.0;
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    const double distance_threshold = 1.5;
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    double res = 0.1;
    int dim = 60/res;
};

#endif 
