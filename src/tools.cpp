#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Calculate the RMSE.
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  // Check the validity of the inputs:
  //  1. the estimation vector size should not be zero
  if (estimations.size() == 0){
    throw "Input error - Input vector size is zero!";
  }
  
  //  2. the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()){
    throw "Input error - Input vectors have different sizes!";
  }

  VectorXd res_sum(4);
  res_sum << 0,0,0,0;
  
  for(int i=0; i < estimations.size(); ++i){

    VectorXd res = (estimations[i] - ground_truth[i]);
    res = res.array()*res.array();
    res_sum = res_sum + res;
  }

  //calculate the mean
  res_sum = res_sum / estimations.size();
  //calculate the squared root
  rmse = res_sum.array().sqrt();

  //return the result
  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Calculate the Jacobian
  MatrixXd Hj(3,4);
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if (px == 0 && py == 0){
    throw "CalculateJacobian() - Error - Division by zero";
  }

  // compute the Jacobian matrix
  float sqr_sum = px*px + py*py;
  float nom1 = sqrt(sqr_sum);
  float nom2 = sqrt(sqr_sum*sqr_sum*sqr_sum);
  
  Hj << px/nom1, py/nom1, 0, 0,
       -py/sqr_sum, px/sqr_sum, 0, 0,
        py*(vx*py - vy*px)/nom2, px*(vy*px - vx*py)/nom2, px/nom1, py/nom1;

  return Hj;
}