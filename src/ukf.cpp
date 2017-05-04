#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const double EPSILON = 0.00001;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // TODO(Olala): tune only the process noise, not the measurement noise
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 1.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  H_ = MatrixXd::Zero(2, n_x_);
  H_(0,0) = 1.0;
  H_(1,1) = 1.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // Skip packages not used
  if (!use_laser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    return;
  }
  if (!use_radar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    return;
  }

  if(!is_initialized_){
    x_.fill(0.0);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float px = ro * cos(phi);
      float py = ro * sin(phi);
      x_(0) = px;
      x_(1) = py;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      x_(0) = px;
      x_(1) = py;
    }
    else { // Ignored unused data
      std::cout << "Tried to initialize with unexpected sensor_type_" << endl;
      assert(false);
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  // Predict using current belive and delta_t
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  if (dt > EPSILON){ //only predict when dt is large enough
    previous_timestamp_ = measurement_pack.timestamp_;
    Prediction(dt);
  }

  // Update using mearument pack data
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(measurement_pack);
  } else {
    std::cout << "Unexpected sensor_type_" << std::endl;
    assert(false);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double delta_t) {
  // Generate augmented sigmapoints
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2*n_aug_+1);
  GenerateSigmaPoints(Xsig_aug);
  SigmaPointPredictions(Xsig_aug, delta_t);
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd S = H_ * P_ * H_.transpose();
  S(0,0) += std_laspx_ * std_laspx_;
  S(1,1) += std_laspy_ * std_laspy_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K*y;

  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  /* calculate the lidar NIS */
  NIS_laser_ = (z -  z_pred).transpose()* S.inverse() * (z - z_pred);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;

  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2*n_aug_+1);
  VectorXd z_pred = VectorXd::Zero(n_z);
  PredictRadarMeasurement(z_pred, S, Zsig);

  // Update with mearument data

  VectorXd z = meas_package.raw_measurements_;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  //calculate cross correlation matrix
  for (int i = 0; i < 2*n_aug_+1; i++) {      //2n+1 simga points
    VectorXd z_diff = Zsig.col(i) - z_pred;   //residual
    z_diff(1) = NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();    //Kalman gain K;

  VectorXd z_diff = z - z_pred;     //residual
  z_diff(1) = NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  NIS_radar_ = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
}

void UKF::GenerateSigmaPoints(MatrixXd& Xsig_aug) {
  // Initialize augmented state and covariance matrix
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0.0;
  x_aug(n_x_ + 1) = 0.0;

  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();  // square root of P_aug
  L = sqrt(lambda_ + n_aug_) * L;

  Xsig_aug.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + L.col(i);
    Xsig_aug.col(n_aug_+i+1) = x_aug - L.col(i);
  }
}

void UKF::SigmaPointPredictions(MatrixXd& Xsig_aug, const double delta_t) {

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) < EPSILON) {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }
    else {
        px_p = p_x + v/yawd * (sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);

    double v_p = v + nu_a*delta_t;
    double yaw_p = yaw + yawd*delta_t + 0.5*nu_yawdd*delta_t*delta_t;
    double yawd_p = yawd + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = NormalizeAngle(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::PredictRadarMeasurement(VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig) {

    // radar mearument dimension (ro, phi, ro_dot)
    int n_z = 3;

    //transform sigma points into measurement space
    for (int i = 0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
      // extract values for better readibility
      double p_x = Xsig_pred_(0,i);
      double p_y = Xsig_pred_(1,i);
      double v  = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);

      if (fabs(p_x) < EPSILON && fabs(p_y) < EPSILON) {
          p_x = EPSILON;
          p_y = EPSILON;
      }

      double v1 = cos(yaw)*v;
      double v2 = sin(yaw)*v;

      double ro = sqrt(p_x*p_x + p_y*p_y);
      double phi = atan2(p_y, p_x);
      double ro_dot = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);
      Zsig(0,i) = ro;
      Zsig(1,i) = phi;
      Zsig(2,i) = ro_dot;
    }

    //mean predicted measurement
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    for (int i=0; i < 2*n_aug_+1; i++) {  //2n+1 simga points
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;
      z_diff(1) = NormalizeAngle(z_diff(1));

      S += weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd::Zero(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;
    S += R;
}

void UKF::DisplayCurrentState() {
    std::cout << "x_\n" << x_ << std::endl;
    std::cout << "P_\n" << P_ << std::endl;
}

double UKF::NormalizeAngle(const double angle) {
  double ret = fmod((angle + M_PI), 2*M_PI) - M_PI;
  return ret;
}
