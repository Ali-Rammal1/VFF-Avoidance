#include "vff_avoidance/AvoidanceNode.hpp"
#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace vff_avoidance
{

AvoidanceNode::AvoidanceNode() : Node("avoidance_node")
{
  // Declare and get parameters from configuration file
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("debug_marker_topic", "/vff_debug");
  this->declare_parameter<double>("attractive_gain", 1);
  this->declare_parameter<double>("repulsive_gain", 0.35);
  this->declare_parameter<double>("max_linear_speed", 0.113);
  this->declare_parameter<double>("max_angular_speed", 0.28);
  this->declare_parameter<double>("scan_timeout", 2.0);
  // maximum distance to consider for obstacle repulsion
  this->declare_parameter<double>("max_detection_distance", 0.465);

  std::string scan_topic         = this->get_parameter("scan_topic").as_string();
  std::string cmd_vel_topic      = this->get_parameter("cmd_vel_topic").as_string();
  std::string debug_marker_topic = this->get_parameter("debug_marker_topic").as_string();

  // Create subscription to LaserScan
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 10, std::bind(&AvoidanceNode::laser_callback, this, _1));

  // Create publishers for cmd_vel and debug markers
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(debug_marker_topic, 10);

  // Timer to execute the VFF algorithm at 20Hz (every 50ms)
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&AvoidanceNode::compute_vff, this));
}

void AvoidanceNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Store the latest laser scan data
  laser_scan_ = msg;
  //RCLCPP_INFO(this->get_logger(), "Received laser scan message");
}

void AvoidanceNode::compute_vff()
{
  if (!laser_scan_) {
    //RCLCPP_WARN(this->get_logger(), "No laser scan data received yet");
    return;
  }
  
  // Retrieve parameters
  double attractive_gain = this->get_parameter("attractive_gain").as_double();
  double repulsive_gain  = this->get_parameter("repulsive_gain").as_double();
  double max_linear      = this->get_parameter("max_linear_speed").as_double();
  double max_angular     = this->get_parameter("max_angular_speed").as_double();
  double max_detection   = this->get_parameter("max_detection_distance").as_double();

  // Attractive vector: always points forward (x direction), scaled by attractive_gain
  double attractive_x = attractive_gain;
  double attractive_y = 0.0;

  // Repulsive vector: computed from the closest obstacle within max_detection
  double repulsive_x = 0.0;
  double repulsive_y = 0.0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < laser_scan_->ranges.size(); i++) {
    double distance = laser_scan_->ranges[i];
    // Only consider valid readings within the max_detection distance
    if (distance > max_detection || distance < laser_scan_->range_min) {
      continue;
    }
    if (distance < min_distance) {
      min_distance = distance;
      double angle = laser_scan_->angle_min + i * laser_scan_->angle_increment;
      // Multiply by repulsive_gain
      repulsive_x = -repulsive_gain * std::cos(angle) / distance;
      repulsive_y = -repulsive_gain * std::sin(angle) / distance;
    }
  }

  // Resultant vector: sum of attractive and repulsive vectors
  double result_x = attractive_x + repulsive_x;
  double result_y = attractive_y + repulsive_y;

  // Determine speed commands based on the resultant vector
  double linear_speed  = std::sqrt(result_x * result_x + result_y * result_y);
  double angular_speed = std::atan2(result_y, result_x);

  // limit speeds to safe limits
  linear_speed  = std::min(linear_speed, max_linear);
  angular_speed = std::max(std::min(angular_speed, max_angular), -max_angular);

  // Publish cmd_vel message
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = linear_speed;
  cmd.angular.z = angular_speed;
  cmd_pub_->publish(cmd);
  //RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear=%.2f angular=%.2f", linear_speed, angular_speed);

  // Publish debugging markers if there is at least one subscriber
  if (marker_pub_->get_subscription_count() > 0) {
    publish_debug_markers(attractive_x, attractive_y, repulsive_x, repulsive_y, result_x, result_y);
  }
}


void AvoidanceNode::publish_debug_markers(double att_x, double att_y,
                                          double rep_x, double rep_y,
                                          double res_x, double res_y)
{
  visualization_msgs::msg::MarkerArray markers;

  auto create_marker = [this](double x, double y, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "vff_vectors";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.orientation.w = 1.0;
    marker.points.resize(2);
    marker.points[0].x = 0.0;
    marker.points[0].y = 0.0;
    marker.points[0].z = 0.0;
    marker.points[1].x = x;
    marker.points[1].y = y;
    marker.points[1].z = 0.0;
    return marker;
  };

  // Blue arrow for attractive vector
  markers.markers.push_back(create_marker(att_x, att_y, 0, 0.0f, 0.0f, 1.0f));
  // Red arrow for repulsive vector
  markers.markers.push_back(create_marker(rep_x, rep_y, 1, 1.0f, 0.0f, 0.0f));
  // Green arrow for resultant vector
  markers.markers.push_back(create_marker(res_x, res_y, 2, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace vff_avoidance
