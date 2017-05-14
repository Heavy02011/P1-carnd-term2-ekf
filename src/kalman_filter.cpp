#include <math.h>
#include "kalman_filter.h"
//#include "tools.h" // rbx

using Eigen::MatrixXd;
using Eigen::VectorXd;
//using Tools::CalculateJacobian; // rbx

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // set temoprary variables for following calulations
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];

  // calculate polar coordinates
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px); // correction # 2
  double rhodot = (px*vx + py*vy)/rho;
  
  // setup vectors for actual measurement and residuum
  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;
  VectorXd y = z - z_pred;
  
  // perform normalization
  if (y(1) < -M_PI) y(1) = y(1) + 2*M_PI; // correction # 3a
  if (y(1) >  M_PI) y(1) = y(1) - 2*M_PI; // correction # 3b
 
  // perform calculation according to lecture
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
