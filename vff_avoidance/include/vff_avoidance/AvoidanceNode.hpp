#ifndef VFF_AVOIDANCE_AVOIDANCENODE_HPP_
#define VFF_AVOIDANCE_AVOIDANCENODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace vff_avoidance
{

class AvoidanceNode : public rclcpp::Node
{
public:
  AvoidanceNode();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void compute_vff();
  void publish_debug_markers(double att_x, double att_y,
                             double rep_x, double rep_y,
                             double res_x, double res_y);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // <-- NEW: Declare the member to store the latest LaserScan message.
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
};

}  // namespace vff_avoidance

#endif  // VFF_AVOIDANCE_AVOIDANCENODE_HPP_
