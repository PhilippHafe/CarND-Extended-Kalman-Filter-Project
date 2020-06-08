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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  
  //Q Process covariance matrix
  Eigen::MatrixXd Q_ = MatrixXd(4, 4);
  Q_ << 0, 0, 0, 0,
  		0, 0, 0, 0,
  		0, 0, 0, 0,
  		0, 0, 0, 0;
  //P State covariance matrix
  Eigen::MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
  		0, 1, 0, 0,
  		0, 0, 1000, 0,
  		0, 0, 0, 1000;
  
  //F State transition matrix initialize
  Eigen::MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
  		0, 1, 0, 1,
  		0, 0, 1, 0,
  		0, 0, 0, 1;
  
  //H Measurement matrix for Laser
  H_laser_ << 1, 0, 0, 0,
			0, 1, 0, 0;
  
  //Hj Jacobian matrix
  
  // Create initial state vector without values
  Eigen::VectorXd x_in = VectorXd(4);
  
  //Initialize ekf with laser H and R
  ekf_.Init(x_in, P_, F_, H_laser_, R_laser_, Q_); 
  
  //Noise components
  noise_ax = 9;
  noise_ay = 9;

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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      ekf_.x_ << rho*cos(phi), rho*sin(phi), rho_dot*cos(phi), rho_dot*sin(phi); //px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
      			measurement_pack.raw_measurements_(1),
      			0,
      			0;

    }

    // Time stamp initializing
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 0.25*pow(dt,4)*noise_ax, 0, 0.5*pow(dt,3)*noise_ax, 0,
            0, 0.25*pow(dt,4)*noise_ay, 0, 0.5*pow(dt,3)*noise_ay,
            0.5*pow(dt,3)*noise_ax, 0, pow(dt,2)*noise_ax, 0,
            0, 0.5*pow(dt,3)*noise_ay, 0, pow(dt,2)*noise_ay;
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    // Calculate H for Radar 
	Hj_ = tools.CalculateJacobian(ekf_.x_);	
	ekf_.H_ = Hj_;
	// Set R Matrix 
	ekf_.R_ = R_radar_;
	// Call update function for RADAR
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
