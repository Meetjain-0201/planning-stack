#ifndef PLANNING_STACK__GLOBAL_PLANNER_HPP_
#define PLANNING_STACK__GLOBAL_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner();

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  // Planning timer
  void planningTimerCallback();
  
  // A* algorithm
  nav_msgs::msg::Path planPath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal
  );
  
  // Helper functions
  int worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y);
  void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y);
  bool isValid(int x, int y);
  bool isFree(int x, int y);
  double heuristic(int x1, int y1, int x2, int y2);
  std::vector<std::pair<int, int>> getNeighbors(int x, int y);
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path);
  void inflateCostmap();
  
  // Visualization
  void publishPlanMarkers(const nav_msgs::msg::Path& path);
  
  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr planning_timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<int8_t> inflated_map_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_map_;
  bool has_goal_;
  
  // Parameters
  double inflation_radius_;
  double cost_scaling_factor_;
  int lethal_cost_;
  bool allow_diagonal_;
  double diagonal_cost_;
  double straight_cost_;
  std::string heuristic_type_;
  double heuristic_weight_;
  bool smooth_path_;
  int smoothing_iterations_;
  double smoothing_tolerance_;
  double max_planning_time_;
  double planning_frequency_;
};

#endif  // PLANNING_STACK__GLOBAL_PLANNER_HPP_
