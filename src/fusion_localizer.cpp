#include "fusion_localizer/fusion_localizer.hpp"

FusionLocalizer::FusionLocalizer(const rclcpp::NodeOptions & node_options)
: Node("fusion_localizer", node_options)
{
  timeout_ = this->declare_parameter("timeout", 1.0);

  const double gnss_noise_x = this->declare_parameter("gnss_noise_x", 0.1);
  const double gnss_noise_y = this->declare_parameter("gnss_noise_y", 0.1);
  const double gnss_noise_z = this->declare_parameter("gnss_noise_z", 0.1);
  Eigen::Vector3d gnss_noise;
  gnss_noise << gnss_noise_x, gnss_noise_y, gnss_noise_z;

  const double imu_noise = this->declare_parameter("imu_noise", 0.01);

  kalman_filter_ptr_ = std::make_shared<KalmanFilter>();
  kalman_filter_ptr_->setGnssNoise(gnss_noise_);
  kalman_filter_ptr_->setImuNoise(imu_noise);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 5, std::bind(&FusionLocalizer::imuCallback, this, std::placeholders::_1));
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 5, std::bind(&FusionLocalizer::odometryCallback, this, std::placeholders::_1));
  gnss_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "gnss", 5, std::bind(&FusionLocalizer::gnssCallback, this, std::placeholders::_1));
  pose_with_covariance_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ekf_pose_with_covariance", 5);
  ekf_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("ekf_path", 5);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 * 0.01)),
    std::bind(&FusionLocalizer::timerCallback, this));
}

void FusionLocalizer::timerCallback()
{
  rclcpp::Time current_time_stamp = this->now();

  if (!init_pose_) return;

  geometry_msgs::msg::PoseWithCovarianceStamped ekf_pose_with_covariance;
  Eigen::MatrixXd covariance = kalman_filter_ptr_->getCovariance();

  ekf_pose_with_covariance.header.frame_id = "map";
  ekf_pose_with_covariance.header.stamp = current_time_stamp;
  ekf_pose_with_covariance.pose.pose.position.x = kalman_filter_ptr_->getElement(STATE::X);
  ekf_pose_with_covariance.pose.pose.position.y = kalman_filter_ptr_->getElement(STATE::Y);
  ekf_pose_with_covariance.pose.pose.position.z = kalman_filter_ptr_->getElement(STATE::Z);
  ekf_pose_with_covariance.pose.pose.orientation.x = kalman_filter_ptr_->getElement(STATE::QX);
  ekf_pose_with_covariance.pose.pose.orientation.y = kalman_filter_ptr_->getElement(STATE::QY);
  ekf_pose_with_covariance.pose.pose.orientation.z = kalman_filter_ptr_->getElement(STATE::QZ);
  ekf_pose_with_covariance.pose.pose.orientation.w = kalman_filter_ptr_->getElement(STATE::QW);
  //ekf_pose_with_covariance.pose.covariance[0 * 6 + 0] = covariance(0, 0);
  //ekf_pose_with_covariance.pose.covariance[1 * 6 + 1] = covariance(1, 1);
  //ekf_pose_with_covariance.pose.covariance[5 * 6 + 5] = covariance(2, 2);

  ekf_path_.header.frame_id = "map";
  ekf_path_.header.stamp = current_time_stamp;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose = ekf_pose_with_covariance.pose.pose;
  pose.header.frame_id = "map";
  pose.header.stamp = current_time_stamp;
  ekf_path_.poses.emplace_back(pose);
#if 1
  // publish tf
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "gnss";
  transform_stamped.transform.translation.x = ekf_pose_with_covariance.pose.pose.position.x;
  transform_stamped.transform.translation.y = ekf_pose_with_covariance.pose.pose.position.y;
  transform_stamped.transform.translation.z = ekf_pose_with_covariance.pose.pose.position.z;
  transform_stamped.transform.rotation.x = ekf_pose_with_covariance.pose.pose.orientation.x;
  transform_stamped.transform.rotation.y = ekf_pose_with_covariance.pose.pose.orientation.y;
  transform_stamped.transform.rotation.z = ekf_pose_with_covariance.pose.pose.orientation.z;
  transform_stamped.transform.rotation.w = ekf_pose_with_covariance.pose.pose.orientation.w;
  broadcaster_->sendTransform(transform_stamped);
#endif
  ekf_path_publisher_->publish(ekf_path_);
  pose_with_covariance_publisher_->publish(ekf_pose_with_covariance);
}

void FusionLocalizer::odometryCallback(const nav_msgs::msg::Odometry msg)
{
  current_odom_ptr_ = std::make_shared<nav_msgs::msg::Odometry>(msg);
}

void FusionLocalizer::gnssCallback(const geometry_msgs::msg::PoseStamped msg)
{
  current_gnss_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(msg);
  if (!update_gnss_) update_gnss_ = true;

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

  if (!init_pose_) {
    Eigen::VectorXd initial_pose(10);
    initial_pose(0) = current_gnss_pose_ptr_->pose.position.x;
    initial_pose(1) = current_gnss_pose_ptr_->pose.position.y;
    initial_pose(2) = current_gnss_pose_ptr_->pose.position.z;
    initial_pose(3) = 0.0;
    initial_pose(4) = 0.0;
    initial_pose(5) = 0.0;
    initial_pose(6) = current_gnss_pose_ptr_->pose.orientation.w;
    initial_pose(7) = current_gnss_pose_ptr_->pose.orientation.x;
    initial_pose(8) = current_gnss_pose_ptr_->pose.orientation.y;
    initial_pose(9) = current_gnss_pose_ptr_->pose.orientation.z;
    kalman_filter_ptr_->initialize(initial_pose);
    init_pose_ = true;
  } else {
    //kalman_filter_ptr_->update(z);
  }
  update_gnss_ = false;
}

void FusionLocalizer::imuCallback(const sensor_msgs::msg::Imu msg)
{
  current_imu_ptr_ = std::make_shared<sensor_msgs::msg::Imu>(msg);

  init_pose_ = true;
  if (init_pose_) {
    static auto previous_imu_stamp = current_imu_ptr_->header.stamp;
    double dt = (rclcpp::Time(current_imu_ptr_->header.stamp) - previous_imu_stamp).seconds();

    Eigen::Vector3d acc, gyro;
    acc.x() = current_imu_ptr_->linear_acceleration.x;
    acc.y() = current_imu_ptr_->linear_acceleration.y;
    acc.z() = current_imu_ptr_->linear_acceleration.z;
    gyro.x() = current_imu_ptr_->angular_velocity.x;
    gyro.y() = current_imu_ptr_->angular_velocity.y;
    gyro.z() = current_imu_ptr_->angular_velocity.z;
    kalman_filter_ptr_->predict(acc, gyro, dt);

    previous_imu_stamp = current_imu_ptr_->header.stamp;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(FusionLocalizer)
