#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  // Initialize rmse vector
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // Check for validity of input
  if ((estimations.size() != ground_truth.size()) || estimations.size() == 0) {
    std::cout << "Error in RMSE calculation" << std::endl;
    return rmse;
  }
  //If validity is given, calculate rmse
  for(int i = 0; i < estimations.size(); ++i){
    VectorXd delta = estimations[i] - ground_truth[i];
    delta = delta.array() * delta.array();
    rmse += delta;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  //Initialize variables
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  //Check for validity
  if (px == 0 && py ==0){
      std::cout << "Division by Zero - Jacobian cannot be calculated" <<std::endl;
    return Hj;
  }
  
  //If validity of state is given, calculate Jacobian Matrix
  float C1 = px*px+py*py;
  float C2 = sqrt(C1);
  float C3 = (C1*C2);
  
  Hj << px/C2, py/C2, 0, 0,
  		-py/C1, px/C1, 0, 0,
  		py*(vx*py - vy*px)/C3, px*(px*vy - py*vx)/C3, px/C2, py/C2;
  
  return Hj;
}
