#include "ukf.h"
#include "Eigen/Dense"

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

  // state vector dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  R_lidar_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  // std_a_ = 1;
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // std_yawdd_ = 30;
  std_yawdd_ = M_PI/4;
   
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
  
  is_initialized_ = false;
  previous_timestamp_ = 0.0;
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

}

UKF::~UKF() {}

void UKF::Init(MeasurementPackage meas_package){


  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;


	R_lidar_ << std_laspx_*std_laspx_, 0,
			  0, std_laspy_*std_laspy_;     

	R_radar_ << std_radr_*std_radr_, 0, 0,
			  0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;  

  // augmented state dimension
  n_aug_ = n_x_ + 2;
  lambda_ = 3 -  n_aug_;

  // Initialize weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);     

  // create example matrix with predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  NIS_radar_ = 0.0;
  NIS_lidar_ = 0.0;
  x_.fill(0.0);

  if (meas_package.sensor_type_ ==  MeasurementPackage::RADAR){
    /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    float rho = meas_package.raw_measurements_(0);
    float phi = meas_package.raw_measurements_(1);
    float rho_dot = meas_package.raw_measurements_(2);
    x_(0) = rho * cos(phi);
    x_(1) = rho * sin(phi);

    float vx = rho_dot * cos(phi);
    float vy = rho_dot * sin(phi);
    float v = sqrt(vx*vx + vy*vy);
    x_(3) = v;
  }
  else if (meas_package.sensor_type_ ==  MeasurementPackage::LASER){
    x_(0) = meas_package.raw_measurements_(0);
    x_(1) = meas_package.raw_measurements_(1);
  }

  previous_timestamp_ = meas_package.timestamp_;
  is_initialized_ = true;
  return;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_){
    Init(meas_package);
  }

  double dt = (meas_package.timestamp_  - previous_timestamp_)/1000000.0; // dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  // Prediction step
  Prediction(dt);
  
  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
    UpdateLidar(meas_package);
  }

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
    UpdateRadar(meas_package);
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Generate augmented sigma points
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); // create example sigma point matrix

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_ ;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_* std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

 // predict sigma points
  for (int i = 0; i < 2*n_aug_+1; ++i)
  {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else
    {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a*delta_t*delta_t*cos(yaw);
    py_p = py_p + 0.5 * nu_a*delta_t*delta_t*sin(yaw);
    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // predict state mean
  x_.fill(0.0);
  for (int i =0; i< 2*n_aug_+1; ++i)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  P_.fill(0);
  for (int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }  
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  VectorXd z = meas_package.raw_measurements_;
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i =0; i< 2*n_aug_ + 1; ++i)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }      

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  // calculate innovation covariance matrix S
  S.fill(0);
  for (int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R_lidar_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i)
  {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;  
  // update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K * S * K.transpose();

  //Calculate NIS
  NIS_lidar_ = z_diff.transpose()*S.inverse()*z_diff;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  VectorXd z = meas_package.raw_measurements_;
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yawd = Xsig_pred_(4,i);
    double vx = cos(yaw) * v;
    double vy = sin(yaw) * v; 


    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x*vx + p_y*vy)/sqrt(p_x*p_x + p_y*p_y);
   } 

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i =0; i< 2*n_aug_+1; ++i)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  // calculate innovation covariance matrix S
  S.fill(0);
  for (int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  S = S + R_radar_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i)
  {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1)<-M_PI) z_diff(1) += 2.0 * M_PI;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) <-M_PI) x_diff(3) += 2.0 * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.0 * M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.0 * M_PI; 

  // update state mean and covariance matrix
  x_ = x_ + K*z_diff;
  P_ = P_ - K * S * K.transpose();

  //Calculate NIS
  NIS_radar_ = z_diff.transpose()*S.inverse()*z_diff;
}