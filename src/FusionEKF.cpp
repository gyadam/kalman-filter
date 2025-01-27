#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement (H) matrix - laser
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  
  if (!is_initialized_) {
    //Initialize the state ekf_.x_ with the first measurement & Create the covariance matrix

    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1; //

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Convert radar from polar to cartesian coordinates and initialize state
      float rho = measurement_pack.raw_measurements_[0]; // range
      float phi = measurement_pack.raw_measurements_[1]; // bearing
      float rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
      
      // Coordinate convertion from polar to cartesian
      float px = rho * cos(phi); 
      float py = rho * sin(phi);
      
      // Set limits in case of very small initial values:
      if ( px < 0.0001 ) {
        px = 0.0001;
      }
      if ( py < 0.0001 ) {
        py = 0.0001;
      }
      
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      ekf_.x_ << px, py, vx , vy;     
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);
      
      ekf_.x_ << px, py, 0, 0;
    }
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_);
  dt /= 1000000.0; // convert ms to s
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition matrix F according to the new elapsed time
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Update the process noise covariance matrix (Q) using noise_ax, noise_ay = 9.0
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float dt_2 = dt * dt;      // dt^2
  float dt_3 = dt_2 * dt;    // dt^3
  float dt_4 = dt_3 * dt;    // dt^4
  float dt_4_4 = dt_4 / 4;   // dt^4/4
  float dt_3_2 = dt_3 / 2;   // dt^3/2
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
           0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
           dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
           0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;
  
  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // In case of Radar, use Jacobian:
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // In case of Radar, use H matrix
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output (uncomment if necessary)
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
