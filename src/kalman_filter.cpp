#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */

  // KF Prediction step
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  MatrixXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  //new state
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(2, 2);
  P_ = (I - K * H_)* P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd z_pred(3);
  //Manually convert the current state vector into polar coordinates
  z_pred(0) = sqrt(x_(0)*x_(0) + x_(1)*x_(1)); //rho
  z_pred(1) = atan2(x_(1), x_(0)); //phi in range -180 to 180 deg
  z_pred(2) = (x_(0)*x_(2) + x_(1)*x_(3))/z_pred(0); //rho dot
    
  MatrixXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  //new state
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(2, 2);
  P_ = (I - K * H_)* P_;
  
}
