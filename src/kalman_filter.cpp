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
   * predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd y = z-H_*x_;
  UpdateHelper(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */

  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  
  double rho = sqrt(px*px + py*py);
  //check division by zero
  if(rho < .00001) {
    px += .0001;
    py += .0001;
    rho = sqrt(px * px + py * py);
  }
  double phi =  atan2(py,px);
  
  double rho_dot = (px*vx+py*vy)/rho;
  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;
  VectorXd y = z-h;
    
  y(1) = NormalizeAngle(y(1));
  UpdateHelper(y);
}

void KalmanFilter::UpdateHelper(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K =  P_ * Ht * S.inverse();
  
  // Calculate new state
  x_ = x_ + (K * y);
  int state_size = x_.size();
  MatrixXd I = MatrixXd::Identity(state_size, state_size);
  P_ -= K * H_ * P_;
}

double KalmanFilter::NormalizeAngle(double y) {
  while (y > M_PI) {
    y -= (2 * M_PI);
  }
  while (y < -M_PI) {
    y += (2 * M_PI);
  }
  return y;
}

