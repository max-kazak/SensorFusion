#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.0;  // assume max_a = 6m/s^2, std_a = max_a/2

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  // State dim
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3.0 - n_x_;;

  // Weights of sigma points
  weights_ = VectorXd(n_sig_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<n_sig_; ++i)
    weights_(i) = 0.5 / (n_aug_ + lambda_);

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * Make sure you switch between lidar and radar measurements.
   */
   if (!is_initialized_) {

     x_.fill(0.0);

     if (use_laser_ && use_radar_) {
       if (meas_package.sensor_type_==MeasurementPackage::LASER) {
         std::cout << "Initializing using Lidar" << std::endl;

         x_[0] = meas_package.raw_measurements_[0];
         x_[1] = meas_package.raw_measurements_[1];
       }
     }
     else {
       if (use_laser_ && meas_package.sensor_type_==MeasurementPackage::LASER) {
         std::cout << "Initialized by Lidar" << std::endl;

         x_[0] = meas_package.raw_measurements_[0];
         x_[1] = meas_package.raw_measurements_[1];
       }
       else if (use_radar_ && meas_package.sensor_type_==MeasurementPackage::RADAR) {
         std::cout << "Initialized by Radar" << std::endl;

         x_[0] = meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]);
         x_[1] = meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]);
       }
     }

     // Initialize P as Identity matrix and lower initial stddev to 0.3
     P_ = 0.3 * MatrixXd::Identity(5,5);

     is_initialized_ = true;

   }
   else {
     double dt = (meas_package.timestamp_ - time_us_) / 1e6;  // time_us_ should already be initialized

     Prediction(dt);

     if(meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_){
       UpdateLidar(meas_package);
     }
     else if(meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_){
       UpdateRadar(meas_package);
     }
   }

   // update time_us_ for next measurement
   time_us_ = meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  /**
   * Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */

  /**
   * Augment state with noise
   */

  VectorXd x_aug = VectorXd(n_aug_); // Augment the mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);  // Augment the covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  // Gen sigma points
  MatrixXd Xsig = MatrixXd(n_aug_, n_sig_);
  MatrixXd L = P_aug.llt().matrixL();  // square root matrix
  MatrixXd A = sqrt(lambda_ + n_aug_) * L;

  Xsig.col(0) = x_aug; // First column of Xsig is central sigma point
  for (int i=0; i<n_aug_; ++i) {
    Xsig.col(i+1)       = x_aug + A.col(i);
    Xsig.col(i+1+n_aug_) = x_aug - A.col(i);
  }

  /**
   *Predict sigma points
   */

  for (int i = 0; i< n_sig_; ++i) {
    // extract values for better readability
    double p_x = Xsig(0,i);
    double p_y = Xsig(1,i);
    double v = Xsig(2,i);
    double yaw = Xsig(3,i);
    double yawd = Xsig(4,i);
    double nu_a = Xsig(5,i);
    double nu_yawdd = Xsig(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /**
   * Predict mean and covariance
   */

  VectorXd x_p = VectorXd(n_x_);  // vector for predicted state
  MatrixXd P_p = MatrixXd(n_x_, n_x_);  // covariance matrix for prediction

  // predicted state mean
  x_p.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {  // iterate over sigma points
    x_p = x_p + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_p.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_p;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_p = P_p + weights_(i) * x_diff * x_diff.transpose() ;
  }

  x_ = x_p;
  P_ = P_p;

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}
