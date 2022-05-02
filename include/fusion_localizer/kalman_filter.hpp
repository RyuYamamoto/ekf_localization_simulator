#ifndef _KALMAN_FILTER_HPP_
#define _KALMAN_FILTER_HPP_

#include <Eigen/Dense>

enum STATE{ X = 0, Y = 1, YAW = 2 };

class KalmanFilter
{
public:
  KalmanFilter() {}
  ~KalmanFilter() {}

  Eigen::VectorXd getState() { return x_; }
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
    P_ = Eigen::MatrixXd::Identity(initial_pose.size(), initial_pose.size());
  }

  void setModelNoise(const Eigen::VectorXd model_noise) { model_noise_ = model_noise; }
  void setGnssNoise(const Eigen::VectorXd gnss_noise) { gnss_noise_ = gnss_noise; }

  Eigen::VectorXd kinematicsModel(Eigen::VectorXd x, Eigen::VectorXd u, const double dt)
  {
    Eigen::VectorXd diff_pose(x.rows());

    diff_pose(STATE::YAW) = u(1) * dt;
    if (std::fabs(u(1)) < 1e-10) {
      diff_pose(STATE::X) = u(0) * std::cos(x(STATE::YAW)) * dt;
      diff_pose(STATE::Y) = u(0) * std::sin(x(STATE::YAW)) * dt;
    } else {
      diff_pose(STATE::X) =
        (u(0) / u(1)) * (std::sin(x(STATE::YAW) + u(1) * dt) - std::sin(x(STATE::YAW)));
      diff_pose(STATE::Y) =
        (u(0) / u(1)) * (-std::cos(x(STATE::YAW) + u(1) * dt) + std::cos(x(STATE::YAW)));
    }

    return x + diff_pose;
  }

  void update(const Eigen::VectorXd z)
  {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
    Q(STATE::X, STATE::X) = gnss_noise_(STATE::X);
    Q(STATE::Y, STATE::Y) = gnss_noise_(STATE::Y);
    Q(STATE::YAW, STATE::YAW) = gnss_noise_(STATE::YAW);

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3, 3);

    Eigen::MatrixXd K = P_ * H.transpose() * (Q + H * P_ * H.transpose()).inverse();

    x_ = K * (z - x_) + x_;
    P_ = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P_;
  }

  void predictWithImu(const Eigen::Vector3d acc, const Eigen::Vector3d gyro, const double dt)
  {
    if (0.5 < dt) return;
  }

  void predictWithTwist(const Eigen::VectorXd u, const double dt)
  {
    if (0.5 < dt || dt < 1e-08) return;
    const double yaw = x_(STATE::YAW);
    const double dt_yaw = u(1) * dt;

    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(2, 2);
    M(0, 0) = model_noise_(0) * std::fabs(u(0)) / dt + model_noise_(1) * std::fabs(u(1)) / dt;
    M(1, 1) = model_noise_(2) * std::fabs(u(0)) / dt + model_noise_(3) * std::fabs(u(1)) / dt;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 2);
    A(STATE::X, 0) = (std::sin(yaw + dt_yaw) - std::sin(yaw)) / u(1);
    A(STATE::Y, 0) = (-std::cos(yaw + dt_yaw) + std::cos(yaw)) / u(1);
    A(STATE::X, 1) = -u(0) / (u(1) * u(1)) * (std::sin(yaw + dt_yaw) - std::sin(yaw)) +
                     u(0) / dt_yaw * std::cos(yaw + dt_yaw);
    A(STATE::Y, 1) = -u(0) / (u(1) * u(1)) * (-std::cos(yaw + dt_yaw) + std::cos(yaw)) +
                     u(0) / dt_yaw * std::sin(yaw + dt_yaw);
    A(STATE::YAW, 1) = dt;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3, 3);
    F(STATE::X, 2) = u(0) / u(1) * (std::cos(yaw + dt_yaw) - std::cos(yaw));
    F(STATE::Y, 2) = u(0) / u(1) * (std::sin(yaw + dt_yaw) - std::sin(yaw));

    x_ = kinematicsModel(x_, u, dt);
    P_ = F * P_ * F.transpose() + A * M * A.transpose();
  }

private:
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;

  Eigen::VectorXd model_noise_;
  Eigen::VectorXd gnss_noise_;
};

#endif
