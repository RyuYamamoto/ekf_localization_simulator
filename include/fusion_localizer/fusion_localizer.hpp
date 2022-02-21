#ifndef _FUSION_LOCALIZER_HPP
#define _FUSION_LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "fusion_localizer/kalman_filter.hpp"

class FusionLocalizer : public rclcpp::Node
{
public:
  FusionLocalizer(const rclcpp::NodeOptions & node_options);
  ~FusionLocalizer() = default;

private:
  void gnssCallback(const geometry_msgs::msg::PoseStamped msg);
  void odometryCallback(const nav_msgs::msg::Odometry msg);
  void twistCallback(const geometry_msgs::msg::Twist msg);

  void timerCallback();

  void update();
  void predict();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_with_covariance_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time previous_time_stamp_;

  geometry_msgs::msg::Twist::SharedPtr current_twist_ptr_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_gnss_pose_ptr_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_ptr_;

  std::shared_ptr<KalmanFilter> kalman_filter_ptr_;

  double timeout_;

  Eigen::Vector3d gnss_noise_;
  Eigen::Vector4d model_noise_;
};

#endif
