#include "control_node.hpp"
#include <cmath>
#include <algorithm>
#include <numbers> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;  // Lookahead distance
  goal_tolerance_ = 0.5;     // Distance to consider the goal reached
  linear_speed_ = 1;       // Constant forward speed
  path_index_ = 0;
  lookahead_error_ = 0.01;

  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; path_index_ = 0; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop(){
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) return;

 

  geometry_msgs::msg::Point goal_point = current_path_->poses.back().pose.position;
  geometry_msgs::msg::Point robot_point =  robot_odom_->pose.pose.position;
  double dx = goal_point.x - robot_point.x;
  double dy = goal_point.y - robot_point.y;

  if(std::sqrt(pow(dx,2) + pow(dy,2)) < goal_tolerance_){
    auto stop_vel = geometry_msgs::msg::Twist();
    stop_vel.linear.x = 0.0;
    stop_vel.linear.y = 0.0;
    stop_vel.linear.z = 0.0;
    stop_vel.angular.x = 0.0;
    stop_vel.angular.y = 0.0;
    stop_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_vel); 
    current_path_ = NULL;
  }
  else{
    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
      return;  // No valid lookahead point found
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);
    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
  }
}

int sgn(double x){
  if(x < 0) return -1;
  return 1;
}

double dist(double x1, double y1, double x2, double y2){
  return std::sqrt(pow(x2-x1, 2)+ pow(y2-y1, 2));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  bool intersect_found  = false;
  geometry_msgs::msg::PoseStamped lookahead_point;
  for(size_t i=path_index_; i<current_path_->poses.size()-1; ++i){
    geometry_msgs::msg::Point path_point1 = current_path_->poses[i].pose.position;
    geometry_msgs::msg::Point path_point2 = current_path_->poses[i+1].pose.position;
    geometry_msgs::msg::Point robot_point =  robot_odom_->pose.pose.position;
    double x1 = path_point1.x - robot_point.x;
    double y1 = path_point1.y - robot_point.y;
    double x2 = path_point2.x - robot_point.x;
    double y2 = path_point2.y - robot_point.y;
    //RCLCPP_INFO(this->get_logger(), "Path: x1: %f, y1: %f, x2: %f, y2: %f", x1, y1, x2, y2);
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double D = x1 * y2 - x2 * y1;
    double discriminant = std::pow(lookahead_distance_, 2)*std::pow(dr,2)-std::pow(D,2);
    if(discriminant >= 0){
      double ix1 = ((D*dy) + sgn(dy)*dx*std::sqrt(discriminant))/std::pow(dr,2);
      double ix2 = ((D*dy) - sgn(dy)*dx*std::sqrt(discriminant))/std::pow(dr,2);
      double iy1 = (-(D*dx) + std::abs(dy)*std::sqrt(discriminant))/std::pow(dr,2);
      double iy2 = (-(D*dx) - std::abs(dy)*std::sqrt(discriminant))/std::pow(dr,2);
      //RCLCPP_INFO(this->get_logger(), "Intersects: x1: %f, y1: %f, x2: %f, y2: %f", ix1, iy1, ix2, iy2);
      //RCLCPP_INFO(this->get_logger(), "Valid?: %d",  std::min(x1,x2) - lookahead_error_ <= ix1 && ix1 <= std::max(x1,x2) + lookahead_error_  && 
      //std::min(y1,y2) - lookahead_error_ <= iy1 && iy1 <= std::max(y1,y2) + lookahead_error_);
      if(
        std::min(x1,x2) - lookahead_error_ <= ix1 && ix1 <= std::max(x1,x2) + lookahead_error_  && 
        std::min(y1,y2) - lookahead_error_ <= iy1 && iy1 <= std::max(y1,y2) + lookahead_error_
      ){
        intersect_found = true;
        lookahead_point.pose.position.x = ix1 + robot_point.x;
        lookahead_point.pose.position.y = iy1 + robot_point.y;
        path_index_ = i;
        break;
      }

      if(
        std::min(x1,x2) - lookahead_error_ <= ix2 && ix2 <= std::max(x1,x2) + lookahead_error_ && 
        std::min(y1,y2) - lookahead_error_ <= iy2 && iy2 <= std::max(y1,y2) + lookahead_error_ 
     
      ){
        intersect_found = true;
        lookahead_point.pose.position.x = ix2 + robot_point.x;
        lookahead_point.pose.position.y = iy2 + robot_point.y;
        path_index_ = i;
        break;
      }
    }
  }
  if(intersect_found){
    lookahead_point.header.stamp = rclcpp::Clock().now();
    return lookahead_point;
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands
  geometry_msgs::msg::Twist cmd_vel;
  geometry_msgs::msg::Point target_point =  target.pose.position;
  geometry_msgs::msg::Point robot_point =  robot_odom_->pose.pose.position;
  geometry_msgs::msg::Quaternion robot_orientation =  robot_odom_->pose.pose.orientation;
  double target_heading = atan2(target_point.y - robot_point.y, target_point.x - robot_point.x);
  double current_heading = extractYaw(robot_orientation);
  double steering_angle = target_heading - current_heading;
  steering_angle = std::atan2(std::sin(steering_angle), std::cos(steering_angle));
  double angular_vel = 2*steering_angle;
  angular_vel = std::clamp(angular_vel, -1.0, 1.0);
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = angular_vel;
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  // TODO: Implement distance calculation between two points
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // TODO: Implement quaternion to yaw conversion
  tf2::Quaternion q(
    quat.x,
    quat.y,
    quat.z,
    quat.w); 
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
