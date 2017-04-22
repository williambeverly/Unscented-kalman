#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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

  // TODO: Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // TODO: Process noise standard deviation yaw acceleration in rad/s^2
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
  std_radrd_ = 0.3;

	// ------------------------------------------------------------------
	// Added intialisations
	// ------------------------------------------------------------------

	// intialise the intialisation flag to false
	is_initialized_ = false;

	// intialise the state dimension
	n_x_ = 5;

	// initialise the augmented state dimension
	n_aug_ = 7;

	// initialise the spread factor lambda
	lambda_ = 3 - n_aug_;
	
	// initialise the predicted state matrix (augmented)
	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

	weights_ = VectorXd(2 * n_aug_ + 1);

	// set radar measurement noise covariance
	R_ = MatrixXd(3, 3);
	R_ << std_radr_*std_radr_, 0, 0,
				0, std_radphi_*std_radphi_, 0,
				0, 0, std_radrd_*std_radrd_;

	H_laser_ = MatrixXd(2, 5);
	H_laser_ << 1, 0, 0, 0, 0,
							0, 1, 0, 0, 0;

	R_laser_ = MatrixXd(2, 2);
	R_laser_ << std_laspx_*std_laspx_, 0,
							0, std_laspy_*std_laspy_;

	P_ << 1, 0, 0, 0, 0,
				0, 1, 0, 0, 0,
				0, 0, 1, 0, 0,
				0, 0, 0, 1, 0,
				0, 0, 0, 0, 1;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/*
	This is the main process that deals with:
	1. Initialisation upon first measurement
	2. Prediction for sigma points, state and state covariance matrix
	3. Update of state and covariance for Radar or Lidar measurements, as applicable
	*/

	//Perform initialisation if first iteration
	if (!is_initialized_) {

		// initialise the NIS_laser_ and NIS_radar_ to zero to combat first iteration high values that were encountered
		// due to lack of actual value
		NIS_laser_ = 0;
		NIS_radar_ = 0;

		// define a zero replacement value
		double zero_replacement = 0.001;
		/*
		For Unscented Kalman, the state indices correspond to:
		x_(0) = px
		x_(1) = py
		x_(2) = velocity magnitude
		x_(3) = yaw
		x_(4) = yaw rate
		*/

		//define px and py, intialise to zero
		double px = 0;
		double py = 0;

		//Check if the measurement is of RADAR type
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			//if RADAR then convert from Polar to Cart and initialise state
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];

			//now express the measurements in cart as follows:
			//	px and py are the cosine and sine respectively with rho=hypot
			//	velocity is from vehicles perspective, should not be used for initialisation, as state velocity is from tracked object perspective
			//	same applicable to yaw and yaw rate
			//	so only set px and py

			px = rho*cos(phi);
			py = rho*sin(phi);

		}

		//Otherwise check if the measurement is of LASER type
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			//if LASER then data contains x and y only
			//so simply set them directly
			px = meas_package.raw_measurements_[0];
			py = meas_package.raw_measurements_[1];
		}

		//now deal with the occurence of zero for px AND py by replacing with a small, non-zero value
		if ((px == 0) && (py == 0)) {
			px = zero_replacement;
			py = zero_replacement;
		}

		//intialise the state
		x_ << px, py, 0, 0, 0;
		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;
		return; //return from process measurement, so that this only occurs upon initialisation, without predict and update
	}
		
		// calculate the change in time
		double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
		// Execute prediction step
		Prediction(delta_t);

		if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
			// if sensor type is LASER and user_laser_ == true
			UpdateLidar(meas_package);
		}

		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
			// if sensor type is RADAR and user_radar_ == true
			UpdateRadar(meas_package);
		}

		time_us_ = meas_package.timestamp_;
	}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/* Steps involved:
			1. Generate augmented sigma points
			2. Predict new sigma points
			3. Predict mean and covariance of predicted sigma points
		
	*/
	// 1. Generate augmented sigma points portion
	//---------------------------------------------------------
	VectorXd x_aug_ = VectorXd(n_aug_);
	MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);

	//create augmented mean state
	x_aug_.head(5) = x_;
	x_aug_(5) = 0;
	x_aug_(6) = 0;

	//create augmented covariance matrix
	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(n_x_, n_x_) = P_;
	P_aug_(5, 5) = std_a_*std_a_;
	P_aug_(6, 6) = std_yawdd_*std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug_.llt().matrixL();

	//create augmented sigma points
	Xsig_aug_.col(0) = x_aug_;
	for (int i = 0; i < n_aug_; i++)
	{
		Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
	}
	// End of sigma points generation
	//---------------------------------------------------------

	// 2. predict sigma points
	//---------------------------------------------------------
	for (int i = 0; i< 2 * n_aug_ + 1; i++)
	{
		// extract the values for reference purposes in next steps
		double p_x = Xsig_aug_(0, i);
		double p_y = Xsig_aug_(1, i);
		double v = Xsig_aug_(2, i);
		double yaw = Xsig_aug_(3, i);
		double yawd = Xsig_aug_(4, i);
		double nu_a = Xsig_aug_(5, i);
		double nu_yawdd = Xsig_aug_(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
		}
		else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
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

	// End of predict sigma points
	//---------------------------------------------------------

	// 3. Predict mean and covariance of predicted sigma points
	//---------------------------------------------------------
	// set weights
	double weight_0 = lambda_ / (lambda_ + n_aug_);
	weights_(0) = weight_0;
	for (int i = 1; i<2 * n_aug_ + 1; i++) {
		double weight = 0.5 / (n_aug_ + lambda_);
		weights_(i) = weight;
	}

	// predicted state mean
	// ok to use x_ as x_pred_ as x_ has already been utilised for purpose
	x_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}

	// predicted state covariance matrix
	// ok to use P_ as P_ has already been utilised for purpose
	P_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// normalise the angles
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}

	// End of predicted mean and covariance
	//---------------------------------------------------------
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	// Lidar function is linear, sigma points require more processing
	// therefore, stick to EKF


	VectorXd z = meas_package.raw_measurements_;

	VectorXd z_pred = H_laser_ * x_;
	VectorXd z_diff = z - z_pred;
	MatrixXd Ht = H_laser_.transpose(); // define and store for efficiency
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse(); // define and store for efficiency
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * z_diff);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;

	NIS_laser_ = z_diff.transpose() * Si * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	// Radar function non-linear, therefore utilise sigma points with appropriate equations
	// measurement dimensions
	int n_z_ = 3;

	// declare and initialise required matrices/vectors
	// measurement sigma points matrix
	MatrixXd Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);
	// measurement covariance matrix
	MatrixXd S_ = MatrixXd(n_z_, n_z_);
	// cross correlation matrix
	MatrixXd Tc_ = MatrixXd(n_x_, n_z_);
	// measurement prediction
	VectorXd z_pred_ = VectorXd(n_z_);

	//transform sigma points into measurement space
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);
		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig_(0, i) = sqrt(p_x*p_x + p_y*p_y);	//r
		Zsig_(1, i) = atan2(p_y, p_x);	//phi
		Zsig_(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);	//r_dot
	}

	// mean predicted measurement
  z_pred_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
	}

	// measurement covariance matrix S
	S_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		
		//residual
		VectorXd z_diff = Zsig_.col(i) - z_pred_;

		// angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
	}

	// add measurement noise covariance matrix
	S_ = S_ + R_;

	
	// calculate cross correlation matrix
	Tc_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		
		// residual
		VectorXd z_diff = Zsig_.col(i) - z_pred_;
		// angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		Tc_ = Tc_ + weights_(i) * x_diff * z_diff.transpose();
	}

	MatrixXd Si = S_.inverse(); // define and store for efficiency

	//Kalman gain K;
	MatrixXd K = Tc_ * Si;

	//residual
	VectorXd z_diff = meas_package.raw_measurements_ - z_pred_;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S_*K.transpose();

	NIS_radar_ = z_diff.transpose() * Si * z_diff;
}
