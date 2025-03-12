#include "planner_node.hpp"
#include <queue>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <chrono>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
      } else {
          RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
          planPath();
      }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(pow(dx,2) + pow(dy,2)) < 0.5; // Threshold for reaching the goal
}

bool PlannerNode::traversible(CellIndex idx){
  if(idx.x < 0 || idx.x >= dim || idx.y < 0 || idx.y >= dim){
    return false;
  }
  return true;
}

void PlannerNode::planPath() {
  RCLCPP_INFO(this->get_logger(), "Planning Path...");
  if (!goal_received_ || current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
      return;
  }

  // A* Implementation (pseudo-code)

  std::vector<CellIndex> grid_path;
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>  open;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> graph;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  int start_x = int(std::floor(dim/2 + robot_pose_.position.x/ res));
  int start_y = int(std::floor(dim/2 + robot_pose_.position.y/ res));
  int goal_x = int(std::floor(dim/2 + goal_.point.x/ res));
  int goal_y = int(std::floor(dim/2 + goal_.point.y/ res));
  int Dx = std::abs(goal_x - start_x);
  int Dy = std::abs(goal_y - start_y);
  double start_h = (Dx + Dy) + (std::sqrt(2) - 2) * std::min(Dx, Dy);
  AStarNode goal(CellIndex(goal_x, goal_y), 0, 0);
  AStarNode start(CellIndex(start_x, start_y), start_h, start_h);
  g_score[start.index] = 0;

  open.push(start);                                 

  while(!open.empty()){
    AStarNode current = open.top();
    open.pop();
    closed[current.index] = true; 
    
    
    if(current.index == goal.index){
      //then we are done
      RCLCPP_INFO(this->get_logger(), "Path found");
      CellIndex current_index = current.index;
      while(current_index != start.index){
        grid_path.push_back(current_index);
        current_index = graph[current_index];
      }
      grid_path.push_back(start.index);
      reverse(grid_path.begin(), grid_path.end());
      break;
    }

    for(int dx = -1; dx <= 1; ++dx){
      for(int dy = -1; dy <= 1; ++dy){
        CellIndex neighbour_index(current.index.x + dx, current.index.y + dy);
        if((dx != 0 || dy != 0) && traversible(neighbour_index) && !closed[neighbour_index]){
          int neighbour_Dx = std::abs(goal.index.x - neighbour_index.x);
          int neighbour_Dy = std::abs(goal.index.y - neighbour_index.y);
          double neighbour_g = g_score[current.index] + std::sqrt(pow(dx, 2) + pow(dy, 2));
          double neighbour_h = (neighbour_Dx + neighbour_Dy) + (std::sqrt(2) - 2) * std::min(neighbour_Dx, neighbour_Dy);
          if(g_score.count(neighbour_index) == 0 ||  neighbour_g < g_score[neighbour_index]){
            g_score[neighbour_index] = neighbour_g;
            AStarNode neighbour(neighbour_index, neighbour_g + neighbour_h + current_map_.data[neighbour_index.y*dim + neighbour_index.x], neighbour_h);
            graph[neighbour.index] = current.index;
            open.push(neighbour);
          }
        }
      }
    }
  }
  // Compute path using A* on current_map_
  // Fill path.poses with the resulting waypoints.

  for(size_t i=0; i<grid_path.size(); i++){
    geometry_msgs::msg::PoseStamped pose_stamped;
    geometry_msgs::msg::Pose pose;
    pose.position.x = (grid_path[i].x - dim / 2)*res;
    pose.position.y = (grid_path[i].y - dim / 2)*res;
    pose_stamped.header.stamp = rclcpp::Clock().now();
    pose_stamped.header.frame_id = "sim_world";
    pose_stamped.pose = pose; 
    path.poses.push_back(pose_stamped);
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}


