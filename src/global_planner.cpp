#include "planning_stack/global_planner.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

GlobalPlanner::GlobalPlanner()
: Node("global_planner"),
  has_map_(false),
  has_goal_(false)
{
  // Declare parameters
  this->declare_parameter("planning_frequency", 1.0);
  this->declare_parameter("inflation_radius", 0.4);
  this->declare_parameter("cost_scaling_factor", 10.0);
  this->declare_parameter("lethal_cost", 253);
  this->declare_parameter("allow_diagonal", true);
  this->declare_parameter("diagonal_cost", 1.414);
  this->declare_parameter("straight_cost", 1.0);
  this->declare_parameter("heuristic_type", "euclidean");
  this->declare_parameter("heuristic_weight", 1.0);
  this->declare_parameter("smooth_path", true);
  this->declare_parameter("smoothing_iterations", 100);
  this->declare_parameter("smoothing_tolerance", 0.1);
  this->declare_parameter("max_planning_time", 5.0);
  
  // Get parameters
  planning_frequency_ = this->get_parameter("planning_frequency").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
  cost_scaling_factor_ = this->get_parameter("cost_scaling_factor").as_double();
  lethal_cost_ = this->get_parameter("lethal_cost").as_int();
  allow_diagonal_ = this->get_parameter("allow_diagonal").as_bool();
  diagonal_cost_ = this->get_parameter("diagonal_cost").as_double();
  straight_cost_ = this->get_parameter("straight_cost").as_double();
  heuristic_type_ = this->get_parameter("heuristic_type").as_string();
  heuristic_weight_ = this->get_parameter("heuristic_weight").as_double();
  smooth_path_ = this->get_parameter("smooth_path").as_bool();
  smoothing_iterations_ = this->get_parameter("smoothing_iterations").as_int();
  smoothing_tolerance_ = this->get_parameter("smoothing_tolerance").as_double();
  max_planning_time_ = this->get_parameter("max_planning_time").as_double();
  
  // Subscriptions
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&GlobalPlanner::mapCallback, this, std::placeholders::_1)
  );
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal", 10,
    std::bind(&GlobalPlanner::goalCallback, this, std::placeholders::_1)
  );
  
  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/global_plan_markers", 10
  );
  
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Timer for replanning
  planning_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / planning_frequency_),
    std::bind(&GlobalPlanner::planningTimerCallback, this)
  );
  
  RCLCPP_INFO(this->get_logger(), "Global planner initialized");
}

void GlobalPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = msg;
  has_map_ = true;
  
  // Inflate costmap
  inflateCostmap();
  
  RCLCPP_INFO(this->get_logger(), 
    "Map received: %dx%d @ %.2fm resolution",
    map_->info.width, map_->info.height, map_->info.resolution
  );
}


void GlobalPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_ = *msg;
  has_goal_ = true;
  
  RCLCPP_INFO(this->get_logger(), 
    "New goal received: (%.2f, %.2f)",
    goal_.pose.position.x, goal_.pose.position.y
  );
  
  // Plan immediately
  planningTimerCallback();
}

void GlobalPlanner::planningTimerCallback()
{
  if (!has_map_ || !has_goal_) {
    return;
  }
  
  // Get current robot position using TF
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      "map", "base_footprint", tf2::TimePointZero
    );
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    return;
  }
  
  // Extract position from transform
  geometry_msgs::msg::Point start;
  start.x = transform.transform.translation.x;
  start.y = transform.transform.translation.y;
  start.z = 0.0;
  
  RCLCPP_INFO(this->get_logger(), 
    "Robot at (%.2f, %.2f), planning to goal (%.2f, %.2f)",
    start.x, start.y, goal_.pose.position.x, goal_.pose.position.y
  );
  
  // Plan path
  auto path = planPath(start, goal_.pose.position);
  
  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found!");
    return;
  }
  
  // Publish
  path_pub_->publish(path);
  publishPlanMarkers(path);
  
  RCLCPP_INFO(this->get_logger(), 
    "Path published with %zu waypoints", path.poses.size()
  );
}

// Continue to Part 2/3...

// ==================== A* PLANNING ALGORITHM ====================

nav_msgs::msg::Path GlobalPlanner::planPath(
  const geometry_msgs::msg::Point& start,
  const geometry_msgs::msg::Point& goal
)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();
  
  // Convert world to grid coordinates
  int start_x, start_y, goal_x, goal_y;
  if (worldToGrid(start.x, start.y, start_x, start_y) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Start position out of bounds");
    return path;
  }
  if (worldToGrid(goal.x, goal.y, goal_x, goal_y) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Goal position out of bounds");
    return path;
  }
  
  // Check if start and goal are free
  if (!isFree(start_x, start_y)) {
    RCLCPP_ERROR(this->get_logger(), "Start position is in obstacle");
    return path;
  }
  if (!isFree(goal_x, goal_y)) {
    RCLCPP_ERROR(this->get_logger(), "Goal position is in obstacle");
    return path;
  }
  
  // A* data structures
  struct Node {
    int x, y;
    double g, f;
    
    bool operator>(const Node& other) const {
      return f > other.f;
    }
  };
  
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
  std::unordered_map<int, int> came_from;  // key = index, value = parent index
  std::unordered_map<int, double> g_score;
  
  auto getIndex = [this](int x, int y) {
    return y * map_->info.width + x;
  };
  
  // Initialize
  int start_idx = getIndex(start_x, start_y);
  g_score[start_idx] = 0.0;
  double h = heuristic(start_x, start_y, goal_x, goal_y);
  open_set.push({start_x, start_y, 0.0, h});
  
  auto start_time = this->now();
  int iterations = 0;
  
  // A* search
  while (!open_set.empty()) {
    // Timeout check
    if ((this->now() - start_time).seconds() > max_planning_time_) {
      RCLCPP_WARN(this->get_logger(), "Planning timeout after %d iterations", iterations);
      return path;
    }
    
    Node current = open_set.top();
    open_set.pop();
    iterations++;
    
    // Goal reached
    if (current.x == goal_x && current.y == goal_y) {
      RCLCPP_INFO(this->get_logger(), "Path found in %d iterations", iterations);
      
      // Reconstruct path
      std::vector<std::pair<int, int>> grid_path;
      int idx = getIndex(current.x, current.y);
      
      while (came_from.find(idx) != came_from.end()) {
        int x = idx % map_->info.width;
        int y = idx / map_->info.width;
        grid_path.push_back({x, y});
        idx = came_from[idx];
      }
      // Add start
      grid_path.push_back({start_x, start_y});
      
      // Reverse to get start -> goal
      std::reverse(grid_path.begin(), grid_path.end());
      
      // Convert to world coordinates
      for (const auto& [gx, gy] : grid_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        gridToWorld(gx, gy, pose.pose.position.x, pose.pose.position.y);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      
      // Smooth path
      if (smooth_path_ && path.poses.size() > 2) {
        path = smoothPath(path);
      }
      
      return path;
    }
    
    int current_idx = getIndex(current.x, current.y);
    
    // Explore neighbors
    for (const auto& [nx, ny] : getNeighbors(current.x, current.y)) {
      if (!isValid(nx, ny) || !isFree(nx, ny)) {
        continue;
      }
      
      // Calculate cost
      double edge_cost = (nx != current.x && ny != current.y) ? diagonal_cost_ : straight_cost_;
      double tentative_g = g_score[current_idx] + edge_cost;
      
      int neighbor_idx = getIndex(nx, ny);
      
      if (g_score.find(neighbor_idx) == g_score.end() || tentative_g < g_score[neighbor_idx]) {
        came_from[neighbor_idx] = current_idx;
        g_score[neighbor_idx] = tentative_g;
        double h = heuristic(nx, ny, goal_x, goal_y);
        double f = tentative_g + heuristic_weight_ * h;
        open_set.push({nx, ny, tentative_g, f});
      }
    }
  }
  
  RCLCPP_WARN(this->get_logger(), "No path found after %d iterations", iterations);
  return path;
}

// ==================== HELPER FUNCTIONS ====================

int GlobalPlanner::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y)
{
  if (!map_) return -1;
  
  grid_x = static_cast<int>(
    (world_x - map_->info.origin.position.x) / map_->info.resolution
  );
  grid_y = static_cast<int>(
    (world_y - map_->info.origin.position.y) / map_->info.resolution
  );
  
  if (grid_x < 0 || grid_x >= static_cast<int>(map_->info.width) ||
      grid_y < 0 || grid_y >= static_cast<int>(map_->info.height)) {
    return -1;
  }
  
  return 0;
}

void GlobalPlanner::gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y)
{
  world_x = map_->info.origin.position.x + (grid_x + 0.5) * map_->info.resolution;
  world_y = map_->info.origin.position.y + (grid_y + 0.5) * map_->info.resolution;
}

bool GlobalPlanner::isValid(int x, int y)
{
  return x >= 0 && x < static_cast<int>(map_->info.width) &&
         y >= 0 && y < static_cast<int>(map_->info.height);
}

bool GlobalPlanner::isFree(int x, int y)
{
  int idx = y * map_->info.width + x;
  return inflated_map_[idx] < lethal_cost_;
}

double GlobalPlanner::heuristic(int x1, int y1, int x2, int y2)
{
  double dx = static_cast<double>(x2 - x1);
  double dy = static_cast<double>(y2 - y1);
  
  if (heuristic_type_ == "manhattan") {
    return std::abs(dx) + std::abs(dy);
  } else if (heuristic_type_ == "octile") {
    double dmax = std::max(std::abs(dx), std::abs(dy));
    double dmin = std::min(std::abs(dx), std::abs(dy));
    return dmax + (diagonal_cost_ - 1.0) * dmin;
  } else {  // euclidean
    return std::sqrt(dx * dx + dy * dy);
  }
}

std::vector<std::pair<int, int>> GlobalPlanner::getNeighbors(int x, int y)
{
  std::vector<std::pair<int, int>> neighbors;
  
  // 4-connected
  neighbors.push_back({x + 1, y});
  neighbors.push_back({x - 1, y});
  neighbors.push_back({x, y + 1});
  neighbors.push_back({x, y - 1});
  
  // 8-connected (diagonal)
  if (allow_diagonal_) {
    neighbors.push_back({x + 1, y + 1});
    neighbors.push_back({x + 1, y - 1});
    neighbors.push_back({x - 1, y + 1});
    neighbors.push_back({x - 1, y - 1});
  }
  
  return neighbors;
}

// Continue to Part 3/3...

// ==================== PATH SMOOTHING ====================

nav_msgs::msg::Path GlobalPlanner::smoothPath(const nav_msgs::msg::Path& path)
{
  if (path.poses.size() < 3) {
    return path;
  }
  
  nav_msgs::msg::Path smoothed = path;
  
  for (int iter = 0; iter < smoothing_iterations_; ++iter) {
    nav_msgs::msg::Path temp = smoothed;
    
    for (size_t i = 1; i < smoothed.poses.size() - 1; ++i) {
      // Average with neighbors
      temp.poses[i].pose.position.x = 
        0.5 * smoothed.poses[i].pose.position.x + 
        0.25 * smoothed.poses[i-1].pose.position.x + 
        0.25 * smoothed.poses[i+1].pose.position.x;
      
      temp.poses[i].pose.position.y = 
        0.5 * smoothed.poses[i].pose.position.y + 
        0.25 * smoothed.poses[i-1].pose.position.y + 
        0.25 * smoothed.poses[i+1].pose.position.y;
    }
    
    smoothed = temp;
  }
  
  return smoothed;
}

// ==================== COSTMAP INFLATION ====================

void GlobalPlanner::inflateCostmap()
{
  if (!map_) return;
  
  inflated_map_ = map_->data;
  
  int inflation_cells = static_cast<int>(inflation_radius_ / map_->info.resolution);
  
  // Create a copy for inflation
  std::vector<int8_t> temp_map = map_->data;
  
  for (int y = 0; y < static_cast<int>(map_->info.height); ++y) {
    for (int x = 0; x < static_cast<int>(map_->info.width); ++x) {
      int idx = y * map_->info.width + x;
      
      // If this cell is an obstacle
      if (map_->data[idx] >= lethal_cost_) {
        // Inflate around it
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (!isValid(nx, ny)) continue;
            
            int neighbor_idx = ny * map_->info.width + nx;
            double dist = std::sqrt(dx * dx + dy * dy) * map_->info.resolution;
            
            if (dist <= inflation_radius_) {
              // Calculate inflated cost
              int8_t inflated_cost = static_cast<int8_t>(
                lethal_cost_ * std::exp(-cost_scaling_factor_ * dist / inflation_radius_)
              );
              
              temp_map[neighbor_idx] = std::max(temp_map[neighbor_idx], inflated_cost);
            }
          }
        }
      }
    }
  }
  
  inflated_map_ = temp_map;
  
  RCLCPP_INFO(this->get_logger(), "Costmap inflated with radius %.2fm", inflation_radius_);
}

// ==================== VISUALIZATION ====================

void GlobalPlanner::publishPlanMarkers(const nav_msgs::msg::Path& path)
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Path line
  visualization_msgs::msg::Marker line_marker;
  line_marker.header = path.header;
  line_marker.ns = "global_path";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.05;
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;
  line_marker.color.a = 1.0;
  
  for (const auto& pose : path.poses) {
    line_marker.points.push_back(pose.pose.position);
  }
  
  markers.markers.push_back(line_marker);
  
  // Waypoint spheres
  for (size_t i = 0; i < path.poses.size(); ++i) {
    visualization_msgs::msg::Marker sphere;
    sphere.header = path.header;
    sphere.ns = "waypoints";
    sphere.id = static_cast<int>(i);
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose = path.poses[i].pose;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1;
    sphere.color.r = 0.0;
    sphere.color.g = 1.0;
    sphere.color.b = 0.0;
    sphere.color.a = 0.7;
    
    markers.markers.push_back(sphere);
  }
  
  marker_pub_->publish(markers);
}

// ==================== MAIN ====================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
