//
// Created by luka on 22.06.17..
// Kalman filter to estimate disturbances and achieve reference tracking

#ifndef PROJECT_KFDISTURBANCEOBSERVER_H
#define PROJECT_KFDISTURBANCEOBSERVER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace mav_control_attitude {

class KFDisturbanceObserver
{
 private:
  static constexpr int combined_control_mpc_use_ = 1;  // still working with moving masses

  static constexpr int kStateSize = 6 + 2*combined_control_mpc_use_;
  static constexpr int kInputSize = 2 + 2*combined_control_mpc_use_;
  static constexpr int kMeasurementSize = 1;
  static constexpr int kDisturbanceSize = kStateSize;
  static constexpr double kGravity = 9.80665;

  static constexpr int kStateSizeKalman = kStateSize + kDisturbanceSize;
  static constexpr int kMeasurementSizeKalman = kStateSize; // all the states are measurable

 public:
  KFDisturbanceObserver(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  virtual ~KFDisturbanceObserver();

  //Feeding
  // Setting
  void setMeasuredStates(double movable_mass_0_position, double movable_mass_0_speed,
                         double movable_mass_1_position, double movable_mass_1_speed,
                         double rotor_0_speed, double rotor_1_speed,
                         double angle, double angular_velocity)
  {
    measurements_(0) = movable_mass_0_position;
    measurements_(1) = movable_mass_0_speed;
    measurements_(2) = movable_mass_1_position;
    measurements_(3) = movable_mass_1_speed;
    if (combined_control_mpc_use_){
      measurements_(4) = rotor_0_speed;
      measurements_(5) = rotor_1_speed;
    }
    measurements_(4 + 2*combined_control_mpc_use_) = angle;
    measurements_(5 + 2*combined_control_mpc_use_) = angular_velocity;
  }

  void setMovingMassCommand(Eigen::Matrix<double, kInputSize, 1> command)
  {
    command_vector_ = command;
  }

  void setInitialState(double movable_mass_0_position, double movable_mass_0_speed,
                       double movable_mass_1_position, double movable_mass_1_speed,
                       double rotor_0_speed, double rotor_1_speed,
                       double angle, double angular_velocity)
  {
    state_.setZero(); // disturbances set to 0
    state_(0) = movable_mass_0_position;
    state_(1) = movable_mass_0_speed;
    state_(2) = movable_mass_1_position;
    state_(3) = movable_mass_1_speed;
    if (combined_control_mpc_use_) {
      state_(4) = rotor_0_speed;
      state_(5) = rotor_1_speed;
    }
    state_(4 + 2*combined_control_mpc_use_) = angle;
    state_(5 + 2*combined_control_mpc_use_) = angular_velocity;

    // reset state_covariance_
    state_covariance_ = initial_state_covariance_.asDiagonal();
  }

  void setSystemMatrices(Eigen::Matrix<double, kStateSize, kStateSize> model_A,
                         Eigen::Matrix<double, kStateSize, kInputSize> model_B,
                         Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd);

  bool updateEstimator();

  void getEstimatedState(Eigen::VectorXd* estimated_state) const;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:

  ros::NodeHandle nh_, private_nh_;

  typedef Eigen::Matrix<double, kStateSizeKalman, 1> StateVector;
  StateVector state_;
  Eigen::Matrix<double, kMeasurementSizeKalman, 1> measurements_;  // [x1, dx1, x3, dx3, theta, dtheta]
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> state_covariance_; // P0 matrix
  Eigen::Matrix<double, kStateSizeKalman, 1> initial_state_covariance_; // P0 values
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> process_noise_covariance_; // We express it as diag() later.
  Eigen::Matrix<double, kMeasurementSizeKalman, kMeasurementSizeKalman> measurement_covariance_; // We express it as diag() later.

  // Kalman filter matrices of states
  //Eigen::SparseMatrix<double> F_; // System dynamics matrix. // maybe speeds up the process if needed
  Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSizeKalman, kMeasurementSizeKalman> K_; // Kalman gain matrix.
  Eigen::Matrix<double, kMeasurementSizeKalman, kStateSizeKalman> H_; // Measurement matrix.
  Eigen::Matrix<double, kStateSizeKalman, kInputSize> G_;
  //Eigen::SparseMatrix<double> H_; // Measurement matrix.

  // system model
  // Model: A, B, Bd
  // x(k+1) = A*x(k) + B*u(k) + Bd*d(k)
  Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
  Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
  Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd_;  //Disturbance transfer  gas motor matrix

  void calculateKalmanMatrices(Eigen::Matrix<double, kStateSizeKalman, kStateSizeKalman>* F_,
                               Eigen::Matrix<double, kStateSizeKalman, kInputSize>* G_,
                               Eigen::Matrix<double, kMeasurementSizeKalman, kStateSizeKalman>* H_);

  void simulateSystem();
  Eigen::Matrix<double, kInputSize, 1> command_vector_;
  bool initialized_;

};
}


#endif //PROJECT_KFDISTURBANCEOBSERVER_H
