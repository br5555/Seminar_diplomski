//
// Created by luka on 11.06.17..
//

#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_H

#include <morus_control/KFDisturbanceObserver.h>
#include <morus_control/steady_state_calculation.h>

#include "math.h"
#include "geometry_msgs/Vector3.h"
#include "rosgraph_msgs/Clock.h"
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

// CVXGEN solver
#include <morus_control/solver.h>

namespace mav_control_attitude {

    constexpr int combined_control_mpc_use_ = 1;  // still working with moving masses

    // MM_MPC + added variables for CC_MPC
    constexpr int kStateSize = 6 + 2*combined_control_mpc_use_; // [x1, dx1, x3, dx3, theta, dtheta] -> A is [6,6]
    constexpr int kInputSize = 2 + 2*combined_control_mpc_use_; // [x1_ref (m), x3_ref (m)]          -> B is [6,2]
    constexpr int kMeasurementSize = 1;                        // [theta] -> C is [1,6]
    constexpr int kDisturbanceSize = kStateSize;               // disturbances are looked on all states -> B_d is [6,6]

    constexpr int kPredictionHorizonSteps = 20;
    constexpr double kGravity = 9.80665;

class MPCAttitudeController {
 public:
    MPCAttitudeController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~MPCAttitudeController();

    // After dynamic change update the parameters
    void applyParameters();

    // compute control input
    void calculateControlCommand(Eigen::Matrix<double, kInputSize, 1> *control_commands);


    // setters
    void setAngleRef(double angle_sp)
    {
      angle_sp_ = angle_sp;
    }

    void setClock(rosgraph_msgs::Clock clock)
    {
      clock_read_ = clock;
    }

    void setMovingMassState(control_msgs::JointControllerState msg,
                            int number_moving_mass,
                            double gain_reading)
    {
      switch(number_moving_mass){
        case 0 : movable_mass_0_position_ = gain_reading * msg.process_value;
                 movable_mass_0_speed_ =    gain_reading * msg.process_value_dot;
                 break;
        case 1 : movable_mass_1_position_ = gain_reading * msg.process_value;
                 movable_mass_1_speed_ =    gain_reading * msg.process_value_dot;
                 break;
      }
    }

    void setMotorState(double motor_0_speed, double motor_1_speed)
    {
      motor_0_speed_ = motor_0_speed - w_gm_0_; // linearization around hovering speed
      motor_1_speed_ = motor_1_speed - w_gm_0_;

      if (!getControllerName().compare("Pitch controller")){
        std_msgs::Float64MultiArray motor_msg;
        motor_msg.data.clear();
        motor_msg.data.push_back(motor_0_speed_);
        motor_msg.data.push_back(motor_1_speed_);
        // debugging to see the rotor behaviour
        rotor_velocities_linearizes_pub_.publish(motor_msg);
      }
    }

    void setAngleState(double angle)
    {
      angle_ = angle;
    }

    void setAngularVelocityState(double angular_velocity)
    {
    angular_velocity_ = angular_velocity;
  }

    void setIntegratorConstantMPC(double K_I_MPC_angle)
    {
      K_I_MPC_angle_ = K_I_MPC_angle;
    }

    void setPenaltyMovingMasses(double q_p0, double q_v0, double q_p1, double q_v1)
    {
      q_moving_masses_ << q_p0, q_v0, q_p1, q_v1;
    }

    void setPenaltyRotors(double q_omega_0, double q_omega_1)
    {
      q_rotors_ << q_omega_0, q_omega_1;
    }

    void setPenaltyAttitude(double q_theta, double q_omega)
    {
      q_attitude_ << q_theta, q_omega;
    }

    void setPenaltyCommand(double r_0, double r_1, double r_2, double r_3)
    {
      r_command_ << r_0, r_1, r_2, r_3;
    }

    void setPenaltyChangeCommand(double r_delta_0, double r_delta_1, double r_delta_2, double r_delta_3)
    {
      r_delta_command_ << r_delta_0, r_delta_1, r_delta_2, r_delta_3;
    }

    void setControllerName(std::string controller_name)
    {
      controller_name_ = controller_name;
    }


    // getters
    std::string getControllerName()
    {
      return controller_name_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
    // ros node handles
    ros::NodeHandle nh_, private_nh_;

    // publishers for debugging
    ros::Publisher target_state_pub_;
    ros::Publisher target_input_pub_;
    ros::Publisher disturbances_pub_;
    ros::Publisher MPC_solver_status_pub_;
    ros::Publisher rotor_velocities_linearizes_pub_;

    //initialize system
    void initializeParameters();
    bool initialized_parameters_;

    // controller variables
    double angle_sp_;
    rosgraph_msgs::Clock clock_read_;
    // states of the system
    double movable_mass_0_position_;
    double movable_mass_0_speed_;
    double movable_mass_1_position_;
    double movable_mass_1_speed_;
    double motor_0_speed_; // speed of the rotor which increases the angle regulated
    double motor_1_speed_; // speed of the rotor which decreases the angle regulated
    double angle_;
    double angular_velocity_;

    // name
    std::string controller_name_;

    // system model
    // Model: A, B, Bd
    // x(k+1) = A*x(k) + B*u(k) + Bd*d(k)
    Eigen::Matrix<double, kStateSize, kStateSize>       model_A_;   //dynamics matrix
    Eigen::Matrix<double, kStateSize, kInputSize>       model_B_;   //transfer matrix
    Eigen::Matrix<double, kStateSize, kDisturbanceSize> model_Bd_;  //Disturbance transfer  gas motor paramsmatrix

    // quadrotor params with moving masses
    double mass_;        // mass of a movable mass
    double mass_quad_;   // mass of the quadrotor body (including gas motors)
    double M_;           // total mass
    double mi_;          // additional constant for equations
    double cd_;          // drag constant (translational)
    double zr_;          // added coefficient for flapping
    double beta_;        // inclination angle of the motor arms
    double beta_gm_;     // this is additional angle of the gas motor prop w.r.t. the motor arm
    double zm_;          // mass displacement iz z-axis
    double Km_;          // voltage to acceleration
    double lm_;          // mass path maximal length
    double arm_offset_;  // motor arm offset from the origin
    double l_;           // motor arm length
    double Tr_;          // transmission rate
    Eigen::Matrix3d Iq_; // moment of inertia of the quadrotor body (without masses)
    double Iyy_b_;
    double Iyy_;

    double Tgm_;  // time constant
    double w_gm_n_; // rpm to rad/s
    double F_n_;
    double b_gm_f_;
    double b_gm_m_; // lucky guess
    double w_gm_0_;
    double F0_;

    // moving mass dynamics parameters (w_mm and zeta_mm)
    double zeta_mm_;
    double tm_;
    double w_mm_;

    // steady state calculation
    SteadyStateCalculation steady_state_calculation_;

    // state penalty
    Eigen::Vector4d q_moving_masses_;
    Eigen::Vector2d q_rotors_;
    Eigen::Vector2d q_attitude_;

    // control penalty
    Eigen::Matrix<double, kInputSize, 1> r_command_;
    Eigen::Matrix<double, kInputSize, 1> r_delta_command_;

    // debug info
    bool verbose_;
    double solve_time_average_;

    // backup LQR
    Eigen::MatrixXd LQR_K_;
    Eigen::Matrix<double, kInputSize, 1> control_commands_temp_;

    // CVXGEN solver parameters (needed if more MPC's are used)
    Params params_;
    Settings settings_;
    int solver_status_;

    // disturbance observer
    bool enable_offset_free_;
    bool enable_integrator_;
    bool initialized_observer_;
    Eigen::Matrix<double, kMeasurementSize, 1> angle_error_integration_;
    Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances_;
    KFDisturbanceObserver disturbance_observer_;

    // controller gains
    double K_I_MPC_angle_;

    // TODO prebacit u YAML FILE pa da se dobije sa get_param
    // sampling time parameters
    double sampling_time_;
    double prediction_sampling_time_;
};
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_H
