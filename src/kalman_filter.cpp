#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  Eigen::MatrixXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::MatrixXd K_times_H = K * H_;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(K_times_H.rows(), K_times_H.cols());
  P_ = (I - K_times_H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double px2 = px * px;
  double py2 = py * py;
  double px2_plus_py2 = px2 + py2;
  double px2_plus_py2_sqrt = sqrt(px2_plus_py2);

  Eigen::VectorXd H_x = Eigen::VectorXd(z.size());
  H_x(0) = px2_plus_py2_sqrt; // ro
  H_x(1) = atan2(py, px); // theta
  H_x(2) = (px * vx + py * vy) / px2_plus_py2_sqrt; // ro_dot

  std::cout << "ekf_.H_x: " << H_x << std::endl;
  Eigen::MatrixXd y = z - H_x;

  while (y(1) > M_PI) {
      y(1) -= 2*M_PI;
  }
  while (y(1) < -M_PI) {
      y(1) += 2*M_PI;
  }
  std::cout << "ekf_.y: " << y << std::endl;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::MatrixXd K_times_H = K * H_;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(K_times_H.rows(), K_times_H.cols());
  P_ = (I - K_times_H) * P_;
}
