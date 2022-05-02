#ifndef _KALMAN_FILTER_3D_HPP_
#define _KALMAN_FILTER_3D_HPP_

#include <Eigen/Dense>

enum STATE { X = 0, Y = 1, Z = 2, VX = 3, VY = 4, VZ = 5, QW = 6, QX = 7, QY = 8, QZ = 9 };

struct State
{
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond quaternion;
};

class KalmanFilter
{
public:
  KalmanFilter()
  {
    x_.resize(10);
    x_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    state_.position << 0.0, 0.0, 0.0;
    state_.velocity << 0.0, 0.0, 0.0;
    state_.quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    P_ = Eigen::MatrixXd::Identity(9, 9);
  }
  ~KalmanFilter() {}

  State getState() { return state_; }
  Eigen::MatrixXd getCovariance() { return P_; }
  double getElement(unsigned int idx) { return x_(idx); }

  inline double normalize(const double radian)
  {
    double normalize = std::fmod((radian + M_PI), 2 * M_PI) - M_PI;
    if (normalize < -M_PI) normalize += (2 * M_PI);
    return normalize;
  }

  void initialize(const Eigen::VectorXd initial_pose)
  {
    x_ = initial_pose;
    state_.position = initial_pose.segment(STATE::X, 3);
    state_.velocity = initial_pose.segment(STATE::VX, 3);
    state_.quaternion = Eigen::Quaterniond(
      initial_pose(STATE::QW), initial_pose(STATE::QX), initial_pose(STATE::QY),
      initial_pose(STATE::QZ));
    P_ = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  }

  void setImuNoise(const double imu_noise) { imu_noise_ = imu_noise; }
  void setGnssNoise(const Eigen::VectorXd gnss_noise) { gnss_noise_ = gnss_noise; }

  void update(const Eigen::VectorXd z)
  {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
    Q(STATE::X, STATE::X) = gnss_noise_(STATE::X);
    Q(STATE::Y, STATE::Y) = gnss_noise_(STATE::Y);
    Q(STATE::Z, STATE::Z) = gnss_noise_(STATE::Z);

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3, 3);

    Eigen::MatrixXd K = P_ * H.transpose() * (Q + H * P_ * H.transpose()).inverse();

    x_.segment(STATE::X, 3) = K * (z - x_.segment(STATE::X, 3)) + x_.segment(STATE::X, 3);
    P_ = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P_;
  }

  void predict(const Eigen::Vector3d acc, const Eigen::Vector3d gyro, const double dt)
  {
    Eigen::Quaterniond quat_wt = Eigen::AngleAxisd(gyro.x() * dt, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(gyro.y() * dt, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(gyro.z() * dt, Eigen::Vector3d::UnitZ());
    Eigen::MatrixXd rotation_matrix = state_.quaternion.toRotationMatrix();

    // state update
    state_.position =
      state_.position + state_.velocity * dt + 0.5 * (rotation_matrix * acc + gravity_) * dt * dt;
    state_.velocity = state_.velocity + (rotation_matrix * acc + gravity_) * dt;
    state_.quaternion = quat_wt * state_.quaternion;

    x_.segment(STATE::X, 3) = state_.position;
    x_.segment(STATE::VX, 3) = state_.velocity;
    x_.segment(STATE::QW, 4) << state_.quaternion.w(), state_.quaternion.x(), state_.quaternion.y(), state_.quaternion.z();

    // Fx*(Jacobian)
    Eigen::MatrixXd Fx = Eigen::MatrixXd::Identity(9, 9);
    Fx.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d acc_mat;
    acc_mat << 0.0, -acc(2), acc(1), acc(2), 0.0, -acc(0), -acc(1), acc(0), 0.0;
    Fx.block<3, 3>(3, 6) = -1 * rotation_matrix * acc_mat * dt;
    Fx.block<3, 3>(6, 6) = rotation_matrix.transpose() * quat_wt;

    // Fi
    Eigen::MatrixXd Fi = Eigen::MatrixXd::Zero(9, 6);

    // Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6);

    P_ = Fx * P_ * Fx.transpose();
  }

private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  Eigen::Vector3d gravity_{0.0, 0.0, -9.80665};

  State state_;

  double imu_noise_;
  Eigen::VectorXd model_noise_;
  Eigen::VectorXd gnss_noise_;
};

#endif
