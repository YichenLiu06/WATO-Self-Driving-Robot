#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void controlLoop(); 
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();  
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);  
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);  
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

  private:
    robot::ControlCore control_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
 
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
 
    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    double lookahead_distance_;
    double lookahead_error_;
    double goal_tolerance_;
    double linear_speed_;
    int path_index_;
};

#endif
