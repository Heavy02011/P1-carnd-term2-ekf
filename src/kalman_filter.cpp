//#include <iostream>
//using namespace std;

//##### remove above ########################################################
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
  // F --> Fj
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  //MatrixXd Fj = CalculateJacobian(x_);

  //x_ = F_ * x_;
  //MatrixXd Ft = F_.transpose();
  //P_ = F_ * P_ * Ft + Q_;

  // H --> Hj
  //MatrixXd Hj = CalculateJacobian(x_);
  double rho = sqrt(px*px + py*py);
  //void  KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * normalize -pi ... pi
  */
  //double phi = atan(py/px);
  double phi = atan2(py, px); // correction # 2
  //std::cout << "phi: " << phi << endl;
  //if (phi < -M_PI) phi = phi + 2*M_PI; // correction # 3a
  //if (phi >  M_PI) phi = phi - 2*M_PI; // correction # 3b
  //cout << "pi=" << M_PI << endl;
  
  double rhodot = (px*vx + py*vy)/rho;
  
  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;
  VectorXd y = z - z_pred;
  
  if (y(1) < -M_PI) y(1) = y(1) + 2*M_PI; // correction # 3a
  if (y(1) >  M_PI) y(1) = y(1) - 2*M_PI; // correction # 3b
  
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
