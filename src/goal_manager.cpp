#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GoalManager : public rclcpp::Node
{
public:
  GoalManager()
  : Node("goal_manager")
  {
    // Publisher
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
    
    // Subscriber (for RViz clicked point)
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&GoalManager::clickedPointCallback, this, std::placeholders::_1)
    );
    
    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    RCLCPP_INFO(this->get_logger(), "Goal manager initialized");
    RCLCPP_INFO(this->get_logger(), "Click 'Publish Point' in RViz to set goals");
  }

private:
  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // Create goal pose
    geometry_msgs::msg::PoseStamped goal;
    goal.header = msg->header;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position = msg->point;
    goal.pose.orientation.w = 1.0;  // Default orientation
    
    // Publish goal
    goal_pub_->publish(goal);
    
    // Visualize goal
    visualization_msgs::msg::Marker marker;
    marker.header = goal.header;
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = goal.pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    marker_pub_->publish(marker);
    
    RCLCPP_INFO(this->get_logger(), 
      "New goal set at (%.2f, %.2f)", 
      goal.pose.position.x, goal.pose.position.y
    );
  }
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalManager>());
  rclcpp::shutdown();
  return 0;
}
