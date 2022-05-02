#ifndef _FUSION_LOCALIZER_HPP
#define _FUSION_LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "fusion_localizer/kalman_filter_3d.hpp"

class FusionLocalizer : public rclcpp::Node
{
public:
  FusionLocalizer(const rclcpp::NodeOptions & node_options);
  ~FusionLocalizer() = default;

private:
  void gnssCallback(const geometry_msgs::msg::PoseStamped msg);
  void odometryCallback(const nav_msgs::msg::Odometry msg);
  void imuCallback(const sensor_msgs::msg::Imu msg);

  void timerCallback();

  void update();
  void predict();

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_with_covariance_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_path_publisher_;

  nav_msgs::msg::Path ekf_path_;
  nav_msgs::msg::Path dead_reckoning_path_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  Eigen::Vector3d dead_reckoning_pose_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped::SharedPtr current_gnss_pose_ptr_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_ptr_;
  sensor_msgs::msg::Imu::SharedPtr current_imu_ptr_;

  std::shared_ptr<KalmanFilter> kalman_filter_ptr_;

  double timeout_;
  bool init_pose_{false};
  bool update_gnss_{false};

  Eigen::Vector3d gnss_noise_;
  Eigen::Vector4d model_noise_;
};

#endif
