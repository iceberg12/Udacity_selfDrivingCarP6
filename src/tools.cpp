#include <iostream>
#include "tools.h"

#define eps 0.0001

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
VectorXd rmse(4);
rmse << 0,0,0,0;

// check the validity of the following inputs:
//  * the estimation vector size should not be zero
//  * the estimation vector size should equal ground truth vector size
if(estimations.size() != ground_truth.size()
		|| estimations.size() == 0){
	cout << "Invalid estimation or ground_truth data" << endl;
	return rmse;
}

//accumulate squared residuals
for(unsigned int i=0; i < estimations.size(); ++i){

	VectorXd residual = estimations[i] - ground_truth[i];

	//coefficient-wise multiplication
	residual = residual.array()*residual.array();
	rmse += residual;
}

//calculate the mean
rmse = rmse/estimations.size();

//calculate the squared root
rmse = rmse.array().sqrt();

//return the result
return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE
  float mag_1 = sqrt(px*px + py*py);
  if (mag_1 < eps){
  	mag_1 = eps;
  }
  float mag_2 = mag_1*mag_1;
  float mag_3 = mag_2*mag_1;
    
  //check division by zero
  if (mag_1 == 0) {
    std::cout << "Divided by 0";
  }
  else {
  //compute the Jacobian matrix
    Hj << px/mag_1, py/mag_1, 0,0,
      -py/mag_2, px/mag_2, 0,0,
      py*(vx*py - vy*px)/mag_3, px*(vy*px - vx*py)/mag_3, px/mag_1, py/mag_1;
  }
  return Hj;
}
