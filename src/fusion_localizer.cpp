#include "fusion_localizer/fusion_localizer.hpp"

FusionLocalizer::FusionLocalizer(const rclcpp::NodeOptions & node_options)
: Node("fusion_localizer", node_options)
{
  timeout_ = this->declare_parameter("timeout", 1.0);

  const double gnss_noise_x = this->declare_parameter("gnss_noise_x", 0.1);
  const double gnss_noise_y = this->declare_parameter("gnss_noise_y", 0.1);
  const double gnss_noise_z = this->declare_parameter("gnss_noise_z", 0.1);
  gnss_noise_ << gnss_noise_x, gnss_noise_y, gnss_noise_z;

  const double model_noise_vv = this->declare_parameter("model_noise_vv", 0.1);
  const double model_noise_vw = this->declare_parameter("model_noise_vw", 0.1);
  const double model_noise_wv = this->declare_parameter("model_noise_wv", 0.1);
  const double model_noise_ww = this->declare_parameter("model_noise_ww", 0.1);
  model_noise_ << model_noise_vv, model_noise_vw, model_noise_wv, model_noise_ww;

  kalman_filter_ptr_ = std::make_shared<KalmanFilter>(0.01);
  kalman_filter_ptr_->setGnssNoise(gnss_noise_);
  kalman_filter_ptr_->setModelNoise(model_noise_);

  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 5, std::bind(&FusionLocalizer::odometryCallback, this, std::placeholders::_1));
  gnss_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "gnss", 5, std::bind(&FusionLocalizer::gnssCallback, this, std::placeholders::_1));
  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "twist", 5, std::bind(&FusionLocalizer::twistCallback, this, std::placeholders::_1));
  pose_with_covariance_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ekf_pose_with_covariance", 5);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 * 0.01)),
    std::bind(&FusionLocalizer::timerCallback, this));
}

void FusionLocalizer::timerCallback()
{
  rclcpp::Time current_time_stamp = rclcpp::Clock().now();

  // motion model update
  if (current_twist_ptr_ != nullptr) {
    double vx, wz;  // input twist
    vx = current_twist_ptr_->linear.x;
    if (std::fabs(current_twist_ptr_->angular.z) < 1e-05)
      wz = 1e-05;
    else
      wz = current_twist_ptr_->angular.z;
    Eigen::Vector2d u(vx, wz);
    kalman_filter_ptr_->predict(u);
  } else {
    RCLCPP_ERROR(get_logger(), "twist msg is not set.");
  }
  // observation update
  if (current_gnss_pose_ptr_ != nullptr) {
    rclcpp::Time gnss_time_stamp = rclcpp::Time(current_gnss_pose_ptr_->header.stamp);
    Eigen::VectorXd z(3);

    z(0) = current_gnss_pose_ptr_->pose.position.x;
    z(1) = current_gnss_pose_ptr_->pose.position.y;
    double roll, pitch, yaw;
    tf2::Quaternion quat(
      current_gnss_pose_ptr_->pose.orientation.x, current_gnss_pose_ptr_->pose.orientation.y,
      current_gnss_pose_ptr_->pose.orientation.z, current_gnss_pose_ptr_->pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
    z(2) = kalman_filter_ptr_->normalize(yaw);

    kalman_filter_ptr_->update(z);
  } else {
    RCLCPP_ERROR(get_logger(), "gnss pose is not set.");
  }
  // TODO
  if (current_odom_ptr_ != nullptr) {
  }

  geometry_msgs::msg::PoseWithCovarianceStamped ekf_pose_with_covariance;
  Eigen::MatrixXd covariance = kalman_filter_ptr_->getCovariance();

  ekf_pose_with_covariance.header.frame_id = "map";
  ekf_pose_with_covariance.header.stamp = current_time_stamp;
  ekf_pose_with_covariance.pose.pose.position.x = kalman_filter_ptr_->getElement(STATE::X);
  ekf_pose_with_covariance.pose.pose.position.y = kalman_filter_ptr_->getElement(STATE::Y);
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, kalman_filter_ptr_->getElement(STATE::YAW));
  ekf_pose_with_covariance.pose.pose.orientation.x = quat.x();
  ekf_pose_with_covariance.pose.pose.orientation.y = quat.y();
  ekf_pose_with_covariance.pose.pose.orientation.z = quat.z();
  ekf_pose_with_covariance.pose.pose.orientation.w = quat.w();
  ekf_pose_with_covariance.pose.covariance[0 * 6 + 0] = covariance(0, 0);
  ekf_pose_with_covariance.pose.covariance[1 * 6 + 1] = covariance(1, 1);
  ekf_pose_with_covariance.pose.covariance[5 * 6 + 5] = covariance(2, 2);

  pose_with_covariance_publisher_->publish(ekf_pose_with_covariance);

  previous_time_stamp_ = current_time_stamp;
}

void FusionLocalizer::odometryCallback(const nav_msgs::msg::Odometry msg)
{
  current_odom_ptr_ = std::make_shared<nav_msgs::msg::Odometry>(msg);
}

void FusionLocalizer::gnssCallback(const geometry_msgs::msg::PoseStamped msg)
{
  current_gnss_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(msg);
}

void FusionLocalizer::twistCallback(const geometry_msgs::msg::Twist msg)
{
  current_twist_ptr_ = std::make_shared<geometry_msgs::msg::Twist>(msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(FusionLocalizer)
