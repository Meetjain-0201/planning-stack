#include "planning_stack/local_planner.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

LocalPlanner::LocalPlanner()
: Node("local_planner"),
  has_path_(false),
  has_odom_(false),
  has_scan_(false)
{
  // Declare parameters
  this->declare_parameter("controller_frequency", 10.0);
  this->declare_parameter("max_vel_x", 0.5);
  this->declare_parameter("min_vel_x", 0.0);
  this->declare_parameter("max_vel_theta", 1.0);
  this->declare_parameter("min_vel_theta", -1.0);
  this->declare_parameter("acc_lim_x", 0.5);
  this->declare_parameter("acc_lim_theta", 1.0);
  this->declare_parameter("decel_lim_x", -0.5);
  this->declare_parameter("decel_lim_theta", -1.0);
  this->declare_parameter("sim_time", 2.0);
  this->declare_parameter("sim_granularity", 0.05);
  this->declare_parameter("vx_samples", 10);
  this->declare_parameter("vtheta_samples", 20);
  this->declare_parameter("path_distance_bias", 32.0);
  this->declare_parameter("goal_distance_bias", 20.0);
  this->declare_parameter("occdist_scale", 0.02);
  this->declare_parameter("obstacle_distance_threshold", 0.3);
  this->declare_parameter("xy_goal_tolerance", 0.15);
  this->declare_parameter("yaw_goal_tolerance", 0.2);
  this->declare_parameter("publish_local_plan", true);
  this->declare_parameter("publish_trajectories", true);
  
  // Get parameters
  controller_frequency_ = this->get_parameter("controller_frequency").as_double();
  max_vel_x_ = this->get_parameter("max_vel_x").as_double();
  min_vel_x_ = this->get_parameter("min_vel_x").as_double();
  max_vel_theta_ = this->get_parameter("max_vel_theta").as_double();
  min_vel_theta_ = this->get_parameter("min_vel_theta").as_double();
  acc_lim_x_ = this->get_parameter("acc_lim_x").as_double();
  acc_lim_theta_ = this->get_parameter("acc_lim_theta").as_double();
  decel_lim_x_ = this->get_parameter("decel_lim_x").as_double();
  decel_lim_theta_ = this->get_parameter("decel_lim_theta").as_double();
  sim_time_ = this->get_parameter("sim_time").as_double();
  sim_granularity_ = this->get_parameter("sim_granularity").as_double();
  vx_samples_ = this->get_parameter("vx_samples").as_int();
  vtheta_samples_ = this->get_parameter("vtheta_samples").as_int();
  path_distance_bias_ = this->get_parameter("path_distance_bias").as_double();
  goal_distance_bias_ = this->get_parameter("goal_distance_bias").as_double();
  occdist_scale_ = this->get_parameter("occdist_scale").as_double();
  obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold").as_double();
  xy_goal_tolerance_ = this->get_parameter("xy_goal_tolerance").as_double();
  yaw_goal_tolerance_ = this->get_parameter("yaw_goal_tolerance").as_double();
  publish_local_plan_ = this->get_parameter("publish_local_plan").as_bool();
  publish_trajectories_ = this->get_parameter("publish_trajectories").as_bool();
  
  // Subscriptions
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/global_path", 10,
    std::bind(&LocalPlanner::pathCallback, this, std::placeholders::_1)
  );
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&LocalPlanner::odomCallback, this, std::placeholders::_1)
  );
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", qos,
    std::bind(&LocalPlanner::scanCallback, this, std::placeholders::_1)
  );
  
  // Publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  local_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_plan", 10);
  traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/dwa_trajectories", 10
  );
  
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Control timer
  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / controller_frequency_),
    std::bind(&LocalPlanner::controlTimerCallback, this)
  );
  
  RCLCPP_INFO(this->get_logger(), "Local planner initialized");
}

void LocalPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  global_path_ = *msg;
  has_path_ = true;
  
  RCLCPP_INFO(this->get_logger(), 
    "Global path received with %zu waypoints", global_path_.poses.size()
  );
}

void LocalPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_.twist = msg->twist;
  has_odom_ = true;
}

void LocalPlanner::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  current_scan_ = *msg;
  has_scan_ = true;
}

void LocalPlanner::controlTimerCallback()
{
  if (!has_path_ || !has_scan_) {
    // Stop robot
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }
  
  // Get current robot pose using TF
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      "map", "base_footprint", tf2::TimePointZero
    );
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Could not get transform: %s", ex.what());
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }
  
  // Update current odometry from TF
  current_odom_.header.frame_id = "map";
  current_odom_.header.stamp = this->now();
  current_odom_.pose.pose.position.x = transform.transform.translation.x;
  current_odom_.pose.pose.position.y = transform.transform.translation.y;
  current_odom_.pose.pose.position.z = transform.transform.translation.z;
  current_odom_.pose.pose.orientation = transform.transform.rotation;
  
  // Get velocity from odometry topic (keep this subscription for velocity)
  // current_odom_.twist is already populated by odomCallback
  
  // Check if goal reached
  if (isGoalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    has_path_ = false;  // Clear path
    return;
  }
  
  // Compute velocity
  geometry_msgs::msg::Twist cmd_vel = computeVelocity();
  cmd_vel_pub_->publish(cmd_vel);
}

// Continue to Part 2/3...

// ==================== DWA ALGORITHM ====================

geometry_msgs::msg::Twist LocalPlanner::computeVelocity()
{
  geometry_msgs::msg::Twist best_cmd;
  
  // Current velocity
  double current_vx = current_odom_.twist.twist.linear.x;
  double current_vtheta = current_odom_.twist.twist.angular.z;
  
  // Dynamic window (velocities reachable in one time step)
  double dt = 1.0 / controller_frequency_;
  double min_vx = std::max(min_vel_x_, current_vx + decel_lim_x_ * dt);
  double max_vx = std::min(max_vel_x_, current_vx + acc_lim_x_ * dt);
  double min_vtheta = std::max(min_vel_theta_, current_vtheta + decel_lim_theta_ * dt);
  double max_vtheta = std::min(max_vel_theta_, current_vtheta + acc_lim_theta_ * dt);
  
  // Sample velocity space
  std::vector<Trajectory> trajectories;
  double best_cost = std::numeric_limits<double>::max();
  Trajectory best_traj;
  best_traj.vx = 0.0;
  best_traj.vtheta = 0.0;
  
  double vx_step = (max_vx - min_vx) / std::max(1, vx_samples_ - 1);
  double vtheta_step = (max_vtheta - min_vtheta) / std::max(1, vtheta_samples_ - 1);
  
  for (int i = 0; i < vx_samples_; ++i) {
    double vx = min_vx + i * vx_step;
    
    for (int j = 0; j < vtheta_samples_; ++j) {
      double vtheta = min_vtheta + j * vtheta_step;
      
      // Simulate trajectory
      Trajectory traj = simulateTrajectory(vx, vtheta);
      
      // Check collision
      if (checkCollision(traj)) {
        continue;
      }
      
      // Calculate cost
      double cost = calculateCost(traj);
      traj.cost = cost;
      trajectories.push_back(traj);
      
      if (cost < best_cost) {
        best_cost = cost;
        best_traj = traj;
      }
    }
  }
  
  // Visualization
  if (publish_local_plan_ && !best_traj.points.empty()) {
    publishLocalPlan(best_traj);
  }
  if (publish_trajectories_) {
    publishTrajectories(trajectories);
  }
  
  best_cmd.linear.x = best_traj.vx;
  best_cmd.angular.z = best_traj.vtheta;
  
  return best_cmd;
}

LocalPlanner::Trajectory LocalPlanner::simulateTrajectory(double vx, double vtheta)
{
  Trajectory traj;
  traj.vx = vx;
  traj.vtheta = vtheta;
  
  // Current pose
  double x = current_odom_.pose.pose.position.x;
  double y = current_odom_.pose.pose.position.y;
  
  // Extract yaw from quaternion
  tf2::Quaternion q(
    current_odom_.pose.pose.orientation.x,
    current_odom_.pose.pose.orientation.y,
    current_odom_.pose.pose.orientation.z,
    current_odom_.pose.pose.orientation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  // Simulate forward
  int num_steps = static_cast<int>(sim_time_ / sim_granularity_);
  for (int i = 0; i < num_steps; ++i) {
    // Update pose
    x += vx * std::cos(yaw) * sim_granularity_;
    y += vx * std::sin(yaw) * sim_granularity_;
    yaw += vtheta * sim_granularity_;
    
    // Store point
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = 0.0;
    traj.points.push_back(pt);
  }
  
  return traj;
}

bool LocalPlanner::checkCollision(const Trajectory& traj)
{
  if (traj.points.empty()) return true;
  
  // Check each point in trajectory against laser scan
  for (const auto& pt : traj.points) {
    // Transform point to base_link frame
    double dx = pt.x - current_odom_.pose.pose.position.x;
    double dy = pt.y - current_odom_.pose.pose.position.y;
    
    // Extract yaw
    tf2::Quaternion q(
      current_odom_.pose.pose.orientation.x,
      current_odom_.pose.pose.orientation.y,
      current_odom_.pose.pose.orientation.z,
      current_odom_.pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Rotate to robot frame
    double local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
    double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
    
    // Calculate angle and distance
    double angle = std::atan2(local_y, local_x);
    double dist = std::sqrt(local_x * local_x + local_y * local_y);
    
    // Find corresponding laser scan index
    int scan_idx = static_cast<int>(
      (angle - current_scan_.angle_min) / current_scan_.angle_increment
    );
    
    if (scan_idx >= 0 && scan_idx < static_cast<int>(current_scan_.ranges.size())) {
      double scan_range = current_scan_.ranges[scan_idx];
      
      if (scan_range < current_scan_.range_max && dist >= scan_range - obstacle_distance_threshold_) {
        return true;  // Collision
      }
    }
  }
  
  return false;  // No collision
}

double LocalPlanner::calculateCost(const Trajectory& traj)
{
  double path_cost = pathDistanceCost(traj);
  double goal_cost = goalDistanceCost(traj);
  double obs_cost = obstacleDistanceCost(traj);
  
  return path_distance_bias_ * path_cost +
         goal_distance_bias_ * goal_cost +
         occdist_scale_ * obs_cost;
}

double LocalPlanner::pathDistanceCost(const Trajectory& traj)
{
  if (traj.points.empty() || global_path_.poses.empty()) {
    return 1000.0;  // High cost
  }
  
  // Distance from trajectory endpoint to closest path point
  auto& end_pt = traj.points.back();
  double min_dist = std::numeric_limits<double>::max();
  
  for (const auto& pose : global_path_.poses) {
    double dx = pose.pose.position.x - end_pt.x;
    double dy = pose.pose.position.y - end_pt.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    min_dist = std::min(min_dist, dist);
  }
  
  return min_dist;
}

double LocalPlanner::goalDistanceCost(const Trajectory& traj)
{
  if (traj.points.empty() || global_path_.poses.empty()) {
    return 1000.0;
  }
  
  // Distance from trajectory endpoint to goal
  auto& end_pt = traj.points.back();
  auto& goal = global_path_.poses.back().pose.position;
  
  double dx = goal.x - end_pt.x;
  double dy = goal.y - end_pt.y;
  
  return std::sqrt(dx * dx + dy * dy);
}

double LocalPlanner::obstacleDistanceCost(const Trajectory& traj)
{
  double min_clearance = std::numeric_limits<double>::max();
  
  // Find minimum clearance to obstacles
  for (const auto& pt : traj.points) {
    double dx = pt.x - current_odom_.pose.pose.position.x;
    double dy = pt.y - current_odom_.pose.pose.position.y;
    
    tf2::Quaternion q(
      current_odom_.pose.pose.orientation.x,
      current_odom_.pose.pose.orientation.y,
      current_odom_.pose.pose.orientation.z,
      current_odom_.pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    double local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
    double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
    
    double angle = std::atan2(local_y, local_x);
    
    int scan_idx = static_cast<int>(
      (angle - current_scan_.angle_min) / current_scan_.angle_increment
    );
    
    if (scan_idx >= 0 && scan_idx < static_cast<int>(current_scan_.ranges.size())) {
      double scan_range = current_scan_.ranges[scan_idx];
      if (scan_range < current_scan_.range_max) {
        double clearance = scan_range - std::sqrt(local_x * local_x + local_y * local_y);
        min_clearance = std::min(min_clearance, clearance);
      }
    }
  }
  
  // Inverse cost (higher clearance = lower cost)
  if (min_clearance < obstacle_distance_threshold_) {
    return 1000.0;  // Too close
  }
  
  return 1.0 / (min_clearance + 0.01);  // Avoid division by zero
}

bool LocalPlanner::isGoalReached()
{
  if (global_path_.poses.empty()) return false;
  
  auto& goal = global_path_.poses.back().pose;
  auto& current = current_odom_.pose.pose;
  
  // Position error
  double dx = goal.position.x - current.position.x;
  double dy = goal.position.y - current.position.y;
  double position_error = std::sqrt(dx * dx + dy * dy);
  
  // Yaw error
  tf2::Quaternion q_current(
    current.orientation.x, current.orientation.y,
    current.orientation.z, current.orientation.w
  );
  tf2::Quaternion q_goal(
    goal.orientation.x, goal.orientation.y,
    goal.orientation.z, goal.orientation.w
  );
  
  double roll, pitch, yaw_current, yaw_goal;
  tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw_current);
  tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw_goal);
  
  double yaw_error = std::abs(yaw_goal - yaw_current);
  // Normalize to [-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
  yaw_error = std::abs(yaw_error);
  
  return (position_error < xy_goal_tolerance_) && (yaw_error < yaw_goal_tolerance_);
}

// Continue to Part 3/3...

// ==================== VISUALIZATION ====================

void LocalPlanner::publishLocalPlan(const Trajectory& best_traj)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();
  
  for (const auto& pt : best_traj.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = pt;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  
  local_plan_pub_->publish(path);
}

void LocalPlanner::publishTrajectories(const std::vector<Trajectory>& trajectories)
{
  visualization_msgs::msg::MarkerArray markers;
  
  int id = 0;
  for (const auto& traj : trajectories) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "dwa_trajectories";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    
    // Color based on cost (green = low cost, red = high cost)
    double normalized_cost = std::min(1.0, traj.cost / 100.0);
    marker.color.r = normalized_cost;
    marker.color.g = 1.0 - normalized_cost;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    
    for (const auto& pt : traj.points) {
      marker.points.push_back(pt);
    }
    
    markers.markers.push_back(marker);
  }
  
  traj_pub_->publish(markers);
}

// ==================== MAIN ====================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
