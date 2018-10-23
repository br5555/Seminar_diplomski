//
// Created by luka on 22.06.17..
//

#include <morus_control/KFDisturbanceObserver.h>

namespace mav_control_attitude {

constexpr int KFDisturbanceObserver::kStateSizeKalman;
constexpr int KFDisturbanceObserver::kMeasurementSizeKalman;
constexpr int KFDisturbanceObserver::kStateSize;
constexpr int KFDisturbanceObserver::kMeasurementSize;
constexpr int KFDisturbanceObserver::kDisturbanceSize;
constexpr int KFDisturbanceObserver::kInputSize;
constexpr double KFDisturbanceObserver::kGravity;

KFDisturbanceObserver::KFDisturbanceObserver(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_(false)
{
    // P_k
    initial_state_covariance_.setOnes();
    initial_state_covariance_ *= 10.0; // TODO outside parameter
    state_covariance_.setZero();

    // x_k
    state_.setZero();

    // Q_k
    process_noise_covariance_.setZero(); // TODO outside parameter
    process_noise_covariance_(4 +2*combined_control_mpc_use_,4 +2*combined_control_mpc_use_) = 8.0;
    process_noise_covariance_(5 +2*combined_control_mpc_use_,5 +2*combined_control_mpc_use_) = 8.0;

    // R_k
    measurement_covariance_.setIdentity();
    measurement_covariance_ *= 1.5; // TODO outside parameter

    initialized_ = true;
    ROS_INFO("Kalman Filter Initialized!");
}

void KFDisturbanceObserver::calculateKalmanMatrices(Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> *F_,
                                                    Eigen::Matrix<double, kStateSizeKalman, kInputSize> *G_,
                                                    Eigen::Matrix<double, kMeasurementSizeKalman, kStateSizeKalman> *H_)
{
  Eigen::MatrixXd tempF, tempG, tempH;
  tempF.resize(kStateSizeKalman, kStateSizeKalman);
  tempF << model_A_, model_Bd_,
      Eigen::MatrixXd::Zero(kDisturbanceSize, kStateSize), Eigen::MatrixXd::Identity(kDisturbanceSize, kDisturbanceSize);
  (*F_) = tempF;

  tempG.resize(kStateSizeKalman, kInputSize);
  tempG << model_B_, Eigen::MatrixXd::Zero(kDisturbanceSize, kInputSize);
  (*G_) = tempG;

  tempH.resize(kMeasurementSizeKalman, kStateSizeKalman);
  tempH.setIdentity();
  (*H_) = tempH;

}

void KFDisturbanceObserver::setSystemMatrices(Eigen::Matrix<double, kStateSize, kStateSize> model_A,
                                              Eigen::Matrix<double, kStateSize, kInputSize> model_B,
                                              Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd)
{
  this -> model_A_ = model_A;
  this -> model_B_ = model_B;
  this -> model_Bd_ = model_Bd;
  calculateKalmanMatrices(&F_, &G_, &H_);
}

bool KFDisturbanceObserver::updateEstimator() {
  if (!initialized_)
    return false;

  ROS_INFO_ONCE("KF is updated for first time.");

  // P_k^- = F_(k-1) * P_(k-1)^+ * F_(k-1)^T + process_noise_covariance(Q_(k-1))
  state_covariance_ = F_ * state_covariance_ * F_.transpose();
  state_covariance_ += process_noise_covariance_;

  // update the Kalman gain
  Eigen::Matrix<double, kMeasurementSizeKalman, kMeasurementSizeKalman> tmp;
  tmp = H_ * state_covariance_ * H_.transpose() + measurement_covariance_;
  K_ = state_covariance_ * H_.transpose() * tmp.inverse();

  // update system states "state_"
  simulateSystem();

  // update with measurements
  state_ += (K_ * measurements_) - (K_ * H_ * state_);// (measurements_ - H_ * state_);

  //Update covariance
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> helper_var;
  helper_var = Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman>::Identity() - K_ * H_;
  state_covariance_ = helper_var * state_covariance_ * helper_var.transpose()
      + K_ * measurement_covariance_ * K_.transpose();

  //Limits on estimated_disturbances
  if (!state_.allFinite()) {
    ROS_ERROR("The estimated state in KF Disturbance Observer has a non-finite element");
    return false;
  }

  Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances;
  estimated_disturbances = state_.segment(kStateSize, kDisturbanceSize); // disturbances -> need to limit them !!

  // min limits
  Eigen::Matrix<double, kDisturbanceSize, 1> lower_limits;
  if (combined_control_mpc_use_) {
    lower_limits << -0.29, -2.0, -0.29, -2.0, -20, -20, -0.5, -2.0;
  } else {
    lower_limits << -0.29, -2.0, -0.29, -2.0, -0.5, -2.0;
  }
  estimated_disturbances = estimated_disturbances.cwiseMax(lower_limits);

  // max limits
  Eigen::Matrix<double, kDisturbanceSize, 1> upper_limits;
  if (combined_control_mpc_use_) {
    upper_limits << 0.29, 2.0, 0.29, 2.0, 20, 20, 0.5, 2.0;
  } else {
    upper_limits << 0.29, 2.0, 0.29, 2.0, 0.5, 2.0;
  }
  estimated_disturbances = estimated_disturbances.cwiseMin(upper_limits);

  // update state disturbances after the limits
  state_.segment(kStateSize, kDisturbanceSize) << estimated_disturbances;

  return true;
}

void KFDisturbanceObserver::simulateSystem() {
  StateVector old_state;
  old_state = state_;
  state_ = F_ * old_state + G_ * command_vector_;
}

void KFDisturbanceObserver::getEstimatedState(Eigen::VectorXd *estimated_state) const {
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSizeKalman);
  *estimated_state = this->state_;
}

KFDisturbanceObserver::~KFDisturbanceObserver()
{
}

}