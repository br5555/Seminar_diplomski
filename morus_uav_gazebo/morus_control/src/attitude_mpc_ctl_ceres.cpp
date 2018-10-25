// MPC control with moving masses for the MORUS project
// MPC algorithm file
// master thesis
// author: Luka Pevec

#include <morus_control/attitude_mpc_ctl_ceres.h>
using namespace std; 

namespace mav_control_attitude {
    MPCAttitudeController::MPCAttitudeController(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              initialized_parameters_(false),  // after call of the "initializedSystem" it gets to "true"
              initialized_observer_(false),
              enable_integrator_(true),
              enable_offset_free_(true),
              angle_error_integration_(0.0),
              disturbance_observer_(nh, private_nh),
              steady_state_calculation_(nh, private_nh),
              verbose_(false),
              sampling_time_(0.04),
              prediction_sampling_time_(0.04)
    {
     initializeParameters(); // init the system and its parameters

     // debugging publishers
     target_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/target_states/", 1);
     target_input_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/target_input/",  1);
     disturbances_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("mpc/disturbances/",  1);
     MPC_solver_status_pub_ = nh_.advertise<std_msgs::Int64>("mpc/solver_status/", 1);
     rotor_velocities_linearizes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("motor_speed_lin/", 1);
    }

    MPCAttitudeController::~MPCAttitudeController() { }

/**
    Set the matrices of the system dynamics model_A, model_B and model_Bd
    TODO Cound be done with Yaml file !!!
 */
    void MPCAttitudeController::initializeParameters()
    {
        // TODO throw in .yaml file !!!
        mass_ = 1.0;
        mass_quad_ = 30.8;
        M_ = mass_quad_ + 4 * mass_;
        mi_ = mass_ / mass_quad_;
        cd_ = 1.5;
        zr_ = 0.2;
        beta_ = 0;
        beta_gm_ = 0;
        zm_ = 0.05;
        Km_ = 1;

        lm_ = 0.6;
        arm_offset_ = 0.1;
        l_ = lm_ + arm_offset_;
        Tr_ = 100;
        double Iq_xx = 5.5268 + 0.2;
        double Iq_yy = 5.5268 + 0.2;
        double Iq_zz = 6.8854 + 0.4;
        Iq_ = Eigen::Matrix3d::Zero(3,3);
        Iq_(0,0) = Iq_xx;
        Iq_(1,1) = Iq_yy;
        Iq_(2,2) = Iq_zz;
        Iyy_b_ = 5.5268;
        Iyy_ = Iyy_b_ + 2*mass_*pow(lm_/2, 2);

        Tgm_ = 0.25;
        w_gm_n_ = 7000 / 60 * 2*M_PI;
        F_n_ = 25 * kGravity;
        b_gm_f_ = F_n_ / (pow(w_gm_n_,2));
        b_gm_m_ = 0.01;

        w_gm_0_ = sqrt(M_ * kGravity / 4.0 / b_gm_f_);
        F0_ = b_gm_f_ * pow(w_gm_0_,2);

        // true parameters experimental
        /*
        zeta_mm_ = 0.6551;
        w_mm_ = 14.8508;
        */

        // close true parameters proven with simulation
        // tm = 0.21 s, sigma_m = 0.065944
        zeta_mm_ = 0.6544;
        w_mm_ =   19.7845;

        // construct model matrices
        Eigen::MatrixXd A_continous_time(kStateSize, kStateSize);
        A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        Eigen::MatrixXd B_continous_time(kStateSize, kInputSize);
        B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
        Eigen::MatrixXd Bd_continous_time(kStateSize, kDisturbanceSize);
        Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

        if (combined_control_mpc_use_){
          // CC_MPC //
          // states: (x)
          // [x1, dx1, x3, dx3, omega1, omega3, theta, dtheta] -> A is [8,8]
          // input signals: (u)
          // ['x1_ref (m)'; 'x3_ref (m)'; 'd_omega1 (rad/s)'; 'd_omega3 (rad/s)'] -> B is [8,4]
          A_continous_time(0,1) = 1.0;
          A_continous_time(1,0) = -pow(w_mm_,2);
          A_continous_time(1,1) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(2,3) = 1.0;
          A_continous_time(3,2) = -pow(w_mm_,2);
          A_continous_time(3,3) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(4,4) = -1.0/Tgm_;
          A_continous_time(5,5) = -1.0/Tgm_;
          A_continous_time(6,7) = 1.0;
          A_continous_time(7,0) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));
          A_continous_time(7,1) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;
          A_continous_time(7,2) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));
          A_continous_time(7,3) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;
          A_continous_time(7,4) =  2 * b_gm_f_ * w_gm_0_ * lm_ / Iyy_;
          A_continous_time(7,5) = -2 * b_gm_f_ * w_gm_0_ * lm_ / Iyy_;

          B_continous_time(1,0) = pow(w_mm_,2);
          B_continous_time(3,1) = pow(w_mm_,2);
          B_continous_time(4,2) = 1.0 / Tgm_;
          B_continous_time(5,3) = 1.0 / Tgm_;
          B_continous_time(7,0) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;
          B_continous_time(7,1) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;

          // disturbance on every state -> B_d is [8,8]
          Bd_continous_time.setIdentity();

        } else{
          // MM_MPC //
          // states: (x)
          // [x1, dx1, x3, dx3, theta, dtheta] -> A is [6,6]
          // input signals: (u)
          // [x1_ref (m), x3_ref (m)] -> B is [6,2]
          A_continous_time(0,1) = 1.0;
          A_continous_time(1,0) = -pow(w_mm_,2);
          A_continous_time(1,1) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(2,3) = 1.0;
          A_continous_time(3,2) = -pow(w_mm_,2);
          A_continous_time(3,3) = -2.0*zeta_mm_*w_mm_;
          A_continous_time(4,5) = 1.0;
          A_continous_time(5,0) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));      // dtheta =f(x1)
          A_continous_time(5,1) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;                 // dtheta =f(v1)
          A_continous_time(5,2) = mass_ / Iyy_ * (kGravity + ((1.0-4.0*mi_)*zm_*pow(w_mm_,2)));      // dtheta =f(x1)
          A_continous_time(5,3) = 2.0 * mass_*(1.0-4.0*mi_)*zm_*zeta_mm_*w_mm_/Iyy_;                 // dtheta =f(v1)

          B_continous_time(1,0) = pow(w_mm_,2);
          B_continous_time(3,1) = pow(w_mm_,2);
          B_continous_time(5,0) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;
          B_continous_time(5,1) = -mass_ * (1.0-4.0*mi_)*zm_*pow(w_mm_,2) / Iyy_;

          // disturbance on every state -> B_d is [6,6]
          Bd_continous_time.setIdentity();
        }

        // discretization of matrix A
        model_A_ = (prediction_sampling_time_ * A_continous_time).exp();

        // calculate equation (42) from
        // "Model predictive control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System"
        Eigen::MatrixXd integral_exp_A;
        integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
        const int count_integral_A = 100; // TODO see why that size, maybe aproximation of integration

        // discrete integration
        for (int i = 0; i < count_integral_A; i++) {
            integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
                              * prediction_sampling_time_ / count_integral_A;
        }

        // making the discrete matrices B and C
        model_B_ = integral_exp_A * B_continous_time;
        model_Bd_ = integral_exp_A * Bd_continous_time;

        angle_error_integration_.setZero();

        if (verbose_) {
            ROS_INFO_STREAM("A:   \n" << model_A_);
            ROS_INFO_STREAM("B:   \n" << model_B_);
            ROS_INFO_STREAM("B_d: \n" << model_Bd_);
        }

        //Solver initialization
        // YOLO set_defaults(); // Set basic algorithm parameters.
        // YOLO setup_indexing();
        // YOLO settings_ = settings;
        // YOLO params_ = params;
        // YOLO settings_.resid_tol = 0.0001;
        // YOLO settings_.verbose = 0; // don't show every outcome of computation
        // YOLO settings_.max_iters = 8;  // reduce the maximum iteration count, to assure quickness over accuracy.
        //cout << "A" << endl;
        //cout << model_A_ << endl;
        //cout << "B" << endl;
        //cout << model_B_ << endl;
        //cout << "Bd" << endl;
        //cout << model_Bd_ << endl;
        
        // parameters A, B, Bd for CVXGEN set
        // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.A), kStateSize, kStateSize) =        model_A_;
        // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.B), kStateSize, kInputSize) =        model_B_;
        // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.Bd), kStateSize, kDisturbanceSize) = model_Bd_;

        initialized_parameters_ = true;
        ROS_INFO("Linear MPC attitude controller: initialized correctly");
    }

    void MPCAttitudeController::applyParameters()
    {
      



      // init the cost matrices
      Q.setZero();
      Q_final.setZero();
      R.setZero();
      R_delta.setZero();

      if (combined_control_mpc_use_){
        // CC_MPC
        // fill the cost matrices - Q
        Q.block(0, 0, 4, 4) = q_moving_masses_.asDiagonal();
        Q.block(4, 4, 2, 2) = q_rotors_.asDiagonal();
        Q.block(6, 6, 2, 2) = q_attitude_.asDiagonal();

        // fill the cost matrices - R
        R = r_command_.asDiagonal();

        // fill the cost matrices - R_delta
        R_delta = r_delta_command_.asDiagonal();

      } else {
        // MM_MPC
        // fill the cost matrices - Q
        Q.block(0, 0, 4, 4) = q_moving_masses_.asDiagonal();
        Q.block(4, 4, 2, 2) = q_attitude_.asDiagonal();

        Eigen::Matrix<double, 2, 1> temp_r;
        // take only the first two for the moving masses
        temp_r << r_command_(0), r_command_(1);
        // fill the cost matrices - R
        R = temp_r.asDiagonal();

        Eigen::Matrix<double, 2, 1> temp_delta_r;
        // take only the first two for the moving masses
        temp_delta_r << r_delta_command_(0), r_delta_command_(1);
        // fill the cost matrices - R_delta
        R_delta = temp_delta_r.asDiagonal();
      }

      steady_state_calculation_.setRCommand(r_command_);
      steady_state_calculation_.initialize(model_A_, model_B_, model_Bd_);

      //Compute terminal cost - Riccaty equation
      //Q_final(k+1) = Q + A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A);
      Q_final = Q;
      for (int i = 0; i < 1000; i++) {
        Eigen::MatrixXd temp = (model_B_.transpose() * Q_final * model_B_ + R);
        Q_final = model_A_.transpose() * Q_final * model_A_
            - (model_A_.transpose() * Q_final * model_B_) * temp.inverse()
                * (model_B_.transpose() * Q_final * model_A_) + Q;
      }

      // init the backup regulator - LQR
      Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
      LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);

      //cout << "Q"  << endl;
      //cout << Q << endl;
      //cout << "P" << endl;
      //cout << Q_final << endl;
      //cout << "R" << endl;
      //cout << R << endl;
      //cout << "R_delta" << endl;
      //cout << 100*R_delta << endl;
      // parameters Q, P, R, R_delta for CVXGEN set
      // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.Q), kStateSize, kStateSize) = Q;
      // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.P), kStateSize, kStateSize) = Q_final;
      // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.R), kInputSize, kInputSize) = R;
      // YOLO Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params_.R_delta), kInputSize, kInputSize) = 100*R_delta;

      // constraints for CVXGEN solver set
      // state constraints TODO how to make work
      // MM
      /*
      params_.x_max[0] = lm_/2 - 0.01;    // x1
      params_.x_max[1] = 2;               // dx1
      params_.x_max[2] = lm_/2 - 0.01;    // x3
      params_.x_max[3] = 2;               // dx3
      params_.x_max[4] = 1.0;             // theta
      params_.x_max[5] = 40;              // dtheta

      params_.x_min[0] = -params_.x_max[0];
      params_.x_min[1] = -params_.x_max[1];
      params_.x_min[2] = -params_.x_max[2];
      params_.x_min[3] = -params_.x_max[3];
      params_.x_min[4] = -params_.x_max[4];
      params_.x_min[5] = -params_.x_max[5];
      */
      //cout << "constraint " << endl;
      //cout << lm_/2 - 0.01 << endl; 
      // input constraints
      // YOLO params_.u_max[0] = lm_/2 - 0.01;
      // YOLO params_.u_max[1] = lm_/2 - 0.01;
      // YOLO params_.u_min[0] = -params_.u_max[0];
      // YOLO params_.u_min[1] = -params_.u_max[1];
        
      u_max(0,0) =  lm_/2 - 0.01; 
      u_max(1,0) =  lm_/2 - 0.01;
      u_min(0,0) =  -u_max(0,0); 
      u_min(1,0) =  -u_max(1,0); 
      // YOLO params_.du_max[0] = sampling_time_ * 2; // constraint of 2 m/s
      // YOLO params_.du_max[1] = sampling_time_ * 2;
      // YOLO params_.du_min[0] = -params_.du_max[0];
      // YOLO params_.du_min[1] = -params_.du_max[1];

      du_max(0,0) =  sampling_time_ * 2; 
      du_max(1,0) =  sampling_time_ * 2;
      du_min(0,0) =  -du_max(0,0); 
      du_min(1,0) =  -du_max(1,0); 

      if (combined_control_mpc_use_){
        // YOLO params_.u_max[2] = 50;
        // YOLO params_.u_max[3] = 50;
        // YOLO params_.u_min[2] = -params_.u_max[2];
        // YOLO params_.u_min[3] = -params_.u_max[3];
        
        u_max(2,0) =  50; 
        u_max(3,0) =  50;
        u_min(2,0) =  -u_max(2,0); 
        u_min(3,0) =  -u_max(3,0);
        
        // YOLO params_.du_max[2] = 1;
        // YOLO params_.du_max[3] = 1;
        // YOLO params_.du_min[2] = -params_.du_max[2];
        // YOLO params_.du_min[3] = -params_.du_max[3];
        
        du_max(2,0) =  1; 
        du_max(3,0) =  1;
        du_min(2,0) =  -du_max(2,0); 
        du_min(3,0) =  -du_max(3,0);
      }


      ROS_INFO("Linear MPC: Tuning parameters updated...");
      if (verbose_) {
        ROS_INFO_STREAM("diag(Q) = \n" << Q.diagonal().transpose());
        ROS_INFO_STREAM("diag(R) = \n" << R.diagonal().transpose());
        ROS_INFO_STREAM("diag(R_delta) = \n " << R_delta.diagonal());

        ROS_INFO_STREAM("Q_final (terminal cost) = \n" << Q_final);
        ROS_INFO_STREAM("LQR_K_ = \n" << LQR_K_);
        Eigen::Matrix<double, kStateSize, kStateSize> closed_loop_dynamics = model_A_ - model_B_ * LQR_K_;
        ROS_INFO_STREAM("Closed loop dynamics = \n" << closed_loop_dynamics);
        ROS_INFO_STREAM("Closed loop eigenvalues absolute value (needed <1) = \n" << closed_loop_dynamics.eigenvalues().cwiseAbs());
      }
    }

    void MPCAttitudeController::calculateControlCommand(
        Eigen::Matrix<double, kInputSize, 1> *control_commands)
    {
      assert(control_commands != nullptr);
      assert(initialized_parameters_);

      //Declare variables
      Eigen::VectorXd KF_estimated_state;

      // Kalman filter and disturbance observer
      if (!initialized_observer_){
        disturbance_observer_.setInitialState(movable_mass_0_position_, movable_mass_0_speed_,
                                              movable_mass_1_position_, movable_mass_1_speed_,
                                              motor_0_speed_, motor_1_speed_,
                                              angle_, angular_velocity_);
        disturbance_observer_.setSystemMatrices(model_A_, model_B_, model_Bd_);
        initialized_observer_ = true;
      }

      disturbance_observer_.setMeasuredStates(movable_mass_0_position_, movable_mass_0_speed_,
                                              movable_mass_1_position_, movable_mass_1_speed_,
                                              motor_0_speed_, motor_1_speed_,
                                              angle_, angular_velocity_);
      disturbance_observer_.setMovingMassCommand(control_commands_temp_);

      bool observer_update_successful = disturbance_observer_.updateEstimator();

      if (!observer_update_successful){
        // reset observer and its states
        disturbance_observer_.setInitialState(movable_mass_0_position_, movable_mass_0_speed_,
                                              movable_mass_1_position_, movable_mass_1_speed_,
                                              motor_0_speed_, motor_1_speed_,
                                              angle_, angular_velocity_);
      }

      disturbance_observer_.getEstimatedState(&KF_estimated_state);

      if (enable_offset_free_) {
        estimated_disturbances_ = KF_estimated_state.segment(kStateSize, kDisturbanceSize);
        if (!getControllerName().compare("Roll controller") && verbose_){
          ROS_INFO_STREAM("estimated disturbances: \n" << estimated_disturbances_);
        }
      } else {
        estimated_disturbances_.setZero();
      }

      // feedback integration
      if(enable_integrator_){
        Eigen::Matrix<double, kMeasurementSize, 1> angle_error;

        angle_error(0) = angle_sp_ - angle_;

        double antiwindup_ball = 0.4; // TODO magic numbers - if current number too big
        // discrete integrator
        if (angle_error.norm() < antiwindup_ball) {
          angle_error_integration_ += angle_error * sampling_time_;
        } else {
          angle_error_integration_.setZero();
        }

        Eigen::Matrix<double, kMeasurementSize, 1> integration_limits;
        integration_limits(0) = 20.0; // TODO magic numbers - if integration too big
        angle_error_integration_ = angle_error_integration_.cwiseMax(-integration_limits);
        angle_error_integration_ = angle_error_integration_.cwiseMin(integration_limits);

        Eigen::Matrix<double, kDisturbanceSize, kMeasurementSize> K_I_MPC;
        K_I_MPC.Zero();
        K_I_MPC(4 + 2*combined_control_mpc_use_) = K_I_MPC_angle_; // set by dynamic reconfigure
        estimated_disturbances_ -= K_I_MPC * angle_error_integration_;
      };



      // creating the "target_state" and "current_state" variables
      target_state.setZero();
      target_state(4 + 2*combined_control_mpc_use_,0) = angle_sp_;

      current_state(0) = movable_mass_0_position_;
      current_state(1) = movable_mass_0_speed_;
      current_state(2) = movable_mass_1_position_;
      current_state(3) = movable_mass_1_speed_;
      if (combined_control_mpc_use_) {
        current_state(4) = motor_0_speed_;
        current_state(5) = motor_1_speed_;
      }
      current_state(4 + 2*combined_control_mpc_use_) = angle_;
      current_state(5 + 2*combined_control_mpc_use_) = angular_velocity_;

      Eigen::VectorXd ref(kMeasurementSize);
      ref << angle_sp_;

      if (enable_offset_free_){
        steady_state_calculation_.computeSteadyState(estimated_disturbances_, ref,
                                                    &target_state, &target_input);
        // Debugging variables
        if (!getControllerName().compare("Pitch controller")){
          // publish target_state
          std_msgs::Float64MultiArray target_state_msg;
          target_state_msg.data.clear();
          for (int index = 0; index < kStateSize; ++index) {
            target_state_msg.data.push_back(target_state(index));
          }
          target_state_pub_.publish(target_state_msg);

          // publish target_input
          std_msgs::Float64MultiArray target_input_msg;
          target_input_msg.data.clear();
          for (int index = 0; index < kInputSize; ++index) {
            target_input_msg.data.push_back(target_input(index));
          }
          target_input_pub_.publish(target_input_msg);

          // publish disturbances
          std_msgs::Float64MultiArray disturbances_msg;
          disturbances_msg.data.clear();
          for (int index = 0; index < kDisturbanceSize; ++index) {
            disturbances_msg.data.push_back(estimated_disturbances_(index));
          }
          disturbances_pub_.publish(disturbances_msg);
        }
      }

      if (!getControllerName().compare("Pitch controller") && verbose_){
        ROS_INFO_STREAM("target_states = \n" << target_state);
      }
      //cout << "x_ss target state" << endl;
      //cout << target_state << endl;
      //cout << "u_ss target input" << endl;
      //cout << target_input << endl;
      
      // fill in the structure for CVXGEN solver - x_ss[t], u_ss, x_0, d, u_prev
      for (int i = 1; i < 11; i++) {
        // YOLO Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params_.x_ss[i])) = target_state;
      }
      for (int i = 0; i < 10; i++) {
        // YOLO Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params_.u_ss[i])) = target_input;
      }
      //cout << "x0 current state" << endl;
      //cout << current_state << endl;
      //cout << "d estimated dsturbance " << endl;
      //cout << estimated_disturbances_ << endl;
      //cout << "u previous" << endl;
      //cout << control_commands_temp_ << endl;
      // YOLO Eigen::Map<Eigen::Matrix<double, kStateSize,       1>>(const_cast<double*>(params_.x_0))    = current_state;
      // YOLO Eigen::Map<Eigen::Matrix<double, kDisturbanceSize, 1>>(const_cast<double*>(params_.d  ))    = estimated_disturbances_;
      // YOLO Eigen::Map<Eigen::Matrix<double, kInputSize,       1>>(const_cast<double*>(params_.u_prev)) = control_commands_temp_;

      // fill the extern structure for the solver
      // YOLO settings = settings_;
      // YOLO params = params_;

      // solve the problem quadratic problem
      // YOLO solver_status_ = (int) solve();
      // publish the solver status
      std_msgs::Int64 solver_status_msg;
      solver_status_msg.data = 1; // YOLO solver_status_;
      MPC_solver_status_pub_.publish(solver_status_msg);

      control_commands_temp_.setZero(); // reset the msg for input signals
      int YOLO = 1;
      // YOLO if (solver_status_ >= 0){ // solution found
      if (YOLO >= 0){
        if (combined_control_mpc_use_) {
          // CC_MPC has 4 input variables
          // YOLO control_commands_temp_ << vars.u_0[0], vars.u_0[1], vars.u_0[2], vars.u_0[3];
        } else {
          // MM_MPC has 2 input variables
          // YOLO control_commands_temp_ << vars.u_0[0], vars.u_0[1]; // fill the solution for problem
        }
      }
      else { // solution not found -> LQR working
        ROS_WARN("Linear MPC: Solver failed, use LQR backup");
        Eigen::Matrix<double, kInputSize, 1> K_I;
        K_I.setOnes();
        K_I *= 1.5; // TODO magic number to look at, integrator constant

        // CALCULATING FEEDBACK WITH LQR !!!!!!
        error_states = target_state - current_state;
        control_commands_temp_ = LQR_K_ * error_states; // + K_I * angle_error_integration_;
      }

      // command min limits
      Eigen::Matrix<double, kInputSize, 1> lower_limits_roll;
      if (combined_control_mpc_use_) {
        lower_limits_roll << -(lm_/2.0 - 0.01), -(lm_/2.0 - 0.01), -50, -50;
      } else {
        lower_limits_roll << -(lm_/2.0 - 0.01), -(lm_/2.0 - 0.01);
      }
      control_commands_temp_ = control_commands_temp_.cwiseMax(lower_limits_roll);

      // command max limits
      Eigen::Matrix<double, kInputSize, 1> upper_limits_roll;
      if (combined_control_mpc_use_) {
        upper_limits_roll << (lm_/2.0 - 0.01), (lm_/2.0 - 0.01), 50, 50;
      } else {
        upper_limits_roll << (lm_/2.0 - 0.01), (lm_/2.0 - 0.01);
      }
      control_commands_temp_ = control_commands_temp_.cwiseMin(upper_limits_roll);
      *control_commands = control_commands_temp_;
    }
}
