#ifndef PLANNING_STACK__LOCAL_PLANNER_HPP_
#define PLANNING_STACK__LOCAL_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <cmath>

class LocalPlanner : public rclcpp::Node
{
public:
  LocalPlanner();

private:
  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  // Control timer
  void controlTimerCallback();
  
  // DWA algorithm
  geometry_msgs::msg::Twist computeVelocity();
  
  // Helper functions
  struct Trajectory {
    double vx, vtheta;
    std::vector<geometry_msgs::msg::Point> points;
    double cost;
  };
  
  Trajectory simulateTrajectory(double vx, double vtheta);
  bool checkCollision(const Trajectory& traj);
  double calculateCost(const Trajectory& traj);
  double pathDistanceCost(const Trajectory& traj);
  double goalDistanceCost(const Trajectory& traj);
  double obstacleDistanceCost(const Trajectory& traj);
  bool isGoalReached();
  
  // Visualization
  void publishLocalPlan(const Trajectory& best_traj);
  void publishTrajectories(const std::vector<Trajectory>& trajectories);
  
  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // State
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Odometry current_odom_;
  sensor_msgs::msg::LaserScan current_scan_;
  bool has_path_;
  bool has_odom_;
  bool has_scan_;
  
  // Parameters
  double controller_frequency_;
  double max_vel_x_, min_vel_x_;
  double max_vel_theta_, min_vel_theta_;
  double acc_lim_x_, acc_lim_theta_;
  double decel_lim_x_, decel_lim_theta_;
  double sim_time_, sim_granularity_;
  int vx_samples_, vtheta_samples_;
  double path_distance_bias_;
  double goal_distance_bias_;
  double occdist_scale_;
  double obstacle_distance_threshold_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  bool publish_local_plan_;
  bool publish_trajectories_;
};

#endif  // PLANNING_STACK__LOCAL_PLANNER_HPP_
