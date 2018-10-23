#include <morus_control/attitude_mpc_ctl_node.h>

namespace mav_control_attitude {
    MPCAttitudeControllerNode::MPCAttitudeControllerNode(const ros::NodeHandle& nh,
                                                         const ros::NodeHandle& private_nh)
            : nh_(nh),
              private_nh_(private_nh),
              linear_mpc_roll_(nh_, private_nh_),
              linear_mpc_pitch_(nh_, private_nh_),
              attitude_joy_(nh_, private_nh_),
              dyn_config_server_(private_nh_),
              start_flag_(false),  // flag for the first measurement
              automatic_reference_(false),
              verbose_(false)
    {
        // init the readings od moving mass and rotor sensors
        movable_mass_0_position_ = 0.0;
        movable_mass_1_position_ = 0.0;
        movable_mass_2_position_ = 0.0;
        movable_mass_3_position_ = 0.0;
        movable_mass_0_speed_ = 0.0;
        movable_mass_1_speed_ = 0.0;
        movable_mass_2_speed_ = 0.0;
        movable_mass_3_speed_ = 0.0;
        motor_0_speed_ = 0.0;
        motor_1_speed_ = 0.0;
        motor_2_speed_ = 0.0;
        motor_3_speed_ = 0.0;

        euler_sp_.x = 0.0;
        euler_sp_.y = 0.0;
        euler_sp_.z = 0.0;

        linear_mpc_roll_.applyParameters();
        linear_mpc_roll_.setControllerName("Roll controller");

        linear_mpc_pitch_.applyParameters();
        linear_mpc_pitch_.setControllerName("Pitch controller");

        pitch_commands_.setZero();
        roll_commands_.setZero();

        // dynamic reconfigure server
        dynamic_reconfigure::Server<morus_control::MPCAttitudeControllerConfig>::CallbackType f;
        f = boost::bind(&MPCAttitudeControllerNode::DynConfigCallback, this, _1, _2);
        dyn_config_server_.setCallback(f);

        // Publishers  ( nh -> )
        pub_mass0_ = nh_.advertise<std_msgs::Float64>("movable_mass_0_position_controller/command", 1);
        pub_mass1_ = nh_.advertise<std_msgs::Float64>("movable_mass_1_position_controller/command", 1);
        pub_mass2_ = nh_.advertise<std_msgs::Float64>("movable_mass_2_position_controller/command", 1);
        pub_mass3_ = nh_.advertise<std_msgs::Float64>("movable_mass_3_position_controller/command", 1);
        pub_angle_state_ = nh_.advertise<morus_msgs::AngleAndAngularVelocity>("angles", 1);
        pub_rotors_ = nh_.advertise<mav_msgs::Actuators>("/gazebo/command/motor_speed", 1);
        pub_rotors_attitude_ = nh_.advertise<mav_msgs::Actuators>("command/attitude/motor_speed", 1);

        // Subscribers ( nh <- )
        imu_subscriber_ = nh_.subscribe("imu", 1, &MPCAttitudeControllerNode::AhrsCallback, this); // measured values info
        imu_received_ = false;
        mot_vel_ref_subscriber_ = nh_.subscribe("mot_vel_ref", 1, &MPCAttitudeControllerNode::MotVelRefCallback, this);
        euler_ref_subscriber_ = nh_.subscribe("euler_ref", 1, &MPCAttitudeControllerNode::EulerRefCallback, this); // reference for the angles
        clock_subscriber_ = nh_.subscribe("/clock", 1, &MPCAttitudeControllerNode::ClockCallback, this);  // internal clock variable
        // position of mass 0
        movable_mass_0_state_subscriber_= nh_.subscribe("movable_mass_0_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass0Callback, this);
        movable_mass_0_state_received_ = false;
        // position of mass 1
        movable_mass_1_state_subscriber_= nh_.subscribe("movable_mass_1_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass1Callback, this);
        movable_mass_1_state_received_ = false;
        // position of mass 2
        movable_mass_2_state_subscriber_= nh_.subscribe("movable_mass_2_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass2Callback, this);
        movable_mass_2_state_received_ = false;
        // position of mass 3
        movable_mass_3_state_subscriber_= nh_.subscribe("movable_mass_3_position_controller/state", 1, &MPCAttitudeControllerNode::MovingMass3Callback, this);
        movable_mass_3_state_received_ = false;
        // rotors angular velocities
        motor_speed_subscriber_ = nh_.subscribe("motor_speed", 1, &MPCAttitudeControllerNode::MotorSpeedCallback, this);
        motor_speed_received_ = false;
        // rotors angular velocities sent from height controller
        motor_speed_height_subscriber_ = nh_.subscribe("command/height/motor_speed", 1, &MPCAttitudeControllerNode::MotorSpeedHeightCallback, this);
    }

    MPCAttitudeControllerNode::~MPCAttitudeControllerNode() {
    }

    void MPCAttitudeControllerNode::DynConfigCallback(morus_control::MPCAttitudeControllerConfig &config,
                                                      uint32_t level)
    {
        automatic_reference_ = config.aut_ref;

        // integral component init
        linear_mpc_roll_.setIntegratorConstantMPC( config.K_I_MPC);
        linear_mpc_pitch_.setIntegratorConstantMPC(config.K_I_MPC);

        // q_moving_masses setup
        linear_mpc_roll_.setPenaltyMovingMasses( config.q_p0, config.q_v0, config.q_p1, config.q_v1);
        linear_mpc_pitch_.setPenaltyMovingMasses(config.q_p0, config.q_v0, config.q_p1, config.q_v1);

        linear_mpc_roll_.setPenaltyRotors( config.q_rotor_speed_0, config.q_rotor_speed_1);
        linear_mpc_pitch_.setPenaltyRotors(config.q_rotor_speed_0, config.q_rotor_speed_1);

        // q_attitude setup
        linear_mpc_roll_.setPenaltyAttitude( config.q_angle, config.q_angular_velocity);
        linear_mpc_pitch_.setPenaltyAttitude(config.q_angle, config.q_angular_velocity);

        // r_command setup
        linear_mpc_roll_.setPenaltyCommand( config.r_mass_0, config.r_mass_1,config.r_rotor_0, config.r_rotor_1);
        linear_mpc_pitch_.setPenaltyCommand(config.r_mass_0, config.r_mass_1,config.r_rotor_0, config.r_rotor_1);

        // r_delta_command setup
        linear_mpc_roll_.setPenaltyChangeCommand( config.r_delta_mass_0, config.r_delta_mass_1,
                                                  config.r_delta_rotor_0, config.r_delta_rotor_1);
        linear_mpc_pitch_.setPenaltyChangeCommand(config.r_delta_mass_0, config.r_delta_mass_1,
                                                  config.r_delta_rotor_0, config.r_delta_rotor_1);

        // change the adequate matrices
        linear_mpc_roll_.applyParameters();
        linear_mpc_pitch_.applyParameters();
    }

    void MPCAttitudeControllerNode::AhrsCallback(const sensor_msgs::Imu &msg) {
        /// @details AHRS callback. Used to extract roll, pitch, yaw and their rates.
        /// We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        /// @param msg: Type sensor_msgs::Imu

        ROS_INFO_ONCE("MPCAttitudeController got first odometry message.");

        // read the msg
        double qx = msg.orientation.x;
        double qy = msg.orientation.y;
        double qz = msg.orientation.z;
        double qw = msg.orientation.w;

        // conversion quaternion to euler (yaw - pitch - roll)
        euler_mv_.x = atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
        euler_mv_.y =  asin(2 * (qw * qy - qx * qz));
        euler_mv_.z = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);

        // gyro measurements (p,q,r)
        double p = msg.angular_velocity.x;
        double q = msg.angular_velocity.y;
        double r = msg.angular_velocity.z;

        double sx = sin(euler_mv_.x); // sin(roll)
        double cx = cos(euler_mv_.x); // cos(roll)
        double cy = cos(euler_mv_.y); // cos(pitch)
        double ty = tan(euler_mv_.y); // tan(pitch)

        // conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
        euler_rate_mv_.y = cx * q - sx * r;
        euler_rate_mv_.z = sx / cy * q + cx / cy * r;

        // publish states in roll,pitch,jaw format
        morus_msgs::AngleAndAngularVelocity angles_velocities;
        angles_velocities.roll = (float)  euler_mv_.x;
        angles_velocities.pitch = (float) euler_mv_.y;
        angles_velocities.jaw = (float)   euler_mv_.z;

        angles_velocities.roll_dot = (float)  euler_rate_mv_.x;
        angles_velocities.pitch_dot = (float) euler_rate_mv_.y;
        angles_velocities.jaw_dot = (float)   euler_rate_mv_.z;
        angles_velocities.header.stamp = ros::Time::now();

        pub_angle_state_.publish(angles_velocities);

        if (!start_flag_){
            start_flag_ = true;
            // first execution, not to have big jump at the beginning
            calculateCommands();
            publishCommands();
        }

        imu_received_ = true;

        // publish to check if calculation
        if (verbose_) {
          ROS_INFO_STREAM("angles: \n roll: " << euler_mv_.x <<
                                 "\n pitch: " << euler_mv_.y <<
                                   "\n jaw: " << euler_mv_.z);
        }

    }

    void MPCAttitudeControllerNode::MotVelRefCallback(const std_msgs::Float32 &msg) {
        /// @details Referent motor velocity callback. (This should be published by height controller).
        /// @param msg: Type std_msgs::Float32
        w_sp_ = msg.data;
    }

    void MPCAttitudeControllerNode::EulerRefCallback(const geometry_msgs::Vector3 &msg) {
        /// @details Euler ref values callback.
        /// @param msg: Type geometry_msgs::Vector3 (x-roll, y-pitch, z-yaw)
        euler_sp_ = msg;
        linear_mpc_roll_.setAngleRef(msg.x);
        linear_mpc_pitch_.setAngleRef(msg.y);

        if (automatic_reference_){
            setAutomaticReference();
        }
    }

    void MPCAttitudeControllerNode::setAutomaticReference() {
        static ros::Time t_previous = ros::Time::now();
        static double angle_sp_pitch = 0.1;

        ros::Time t0 = ros::Time::now();
        double dt = (t0 - t_previous).toSec();

        if (dt > 7.0){
            angle_sp_pitch *= -1; // change the reference polarity (+, -, +, -, ...)
            t_previous = t0;
        }
        linear_mpc_pitch_.setAngleRef(angle_sp_pitch);
    }

    void MPCAttitudeControllerNode::ClockCallback(const rosgraph_msgs::Clock &msg) {
        /// @param msg
        clock_read_ = msg;
        linear_mpc_roll_.setClock(msg);
        linear_mpc_pitch_.setClock(msg);
    }

    void MPCAttitudeControllerNode::MovingMass0Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_0_position_ = msg.process_value;
      movable_mass_0_speed_ = msg.process_value_dot;
      linear_mpc_pitch_.setMovingMassState(msg, 0, +1.0);
      movable_mass_0_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MovingMass1Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_1_position_ = msg.process_value;
      movable_mass_1_speed_ = msg.process_value_dot;
      linear_mpc_roll_.setMovingMassState(msg, 0, -1.0);
      movable_mass_1_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MovingMass2Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_2_position_ = msg.process_value;
      movable_mass_2_speed_ = msg.process_value_dot;
      linear_mpc_pitch_.setMovingMassState(msg, 1, -1.0);
      movable_mass_2_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MovingMass3Callback(const control_msgs::JointControllerState& msg) {
      movable_mass_3_position_ = msg.process_value;
      movable_mass_3_speed_ = msg.process_value_dot;
      linear_mpc_roll_.setMovingMassState(msg, 1, +1.0);
      movable_mass_3_state_received_ = true;
    }

    void MPCAttitudeControllerNode::MotorSpeedCallback(const mav_msgs::Actuators &msg) {
      motor_0_speed_ = -msg.angular_velocities.data()[0]; // +
      motor_1_speed_ =  msg.angular_velocities.data()[1]; // +
      motor_2_speed_ = -msg.angular_velocities.data()[2]; // +
      motor_3_speed_ =  msg.angular_velocities.data()[3]; // +
      // all the speeds are now positive
      // 1. state - rotor which increases regulated angle, 2. state - decreases
      linear_mpc_pitch_.setMotorState(motor_2_speed_, motor_0_speed_);
      linear_mpc_roll_.setMotorState( motor_1_speed_, motor_3_speed_);
      motor_speed_received_ = true; // acknowledgement of receiving message
    }

    void MPCAttitudeControllerNode::MotorSpeedHeightCallback(const mav_msgs::Actuators &msg) {
      // read the command sent from height controller
      // forward the msg from the hight controller and to the UAV
      mav_msgs::Actuators combined_rotor_command_msg;
      combined_rotor_command_msg.header.stamp = ros::Time::now();
      combined_rotor_command_msg.angular_velocities.clear();
      combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[0] + pitch_commands_[2]);
      combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[1] + roll_commands_[3]);
      combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[2] + pitch_commands_[3]);
      combined_rotor_command_msg.angular_velocities.push_back(msg.angular_velocities[3] + roll_commands_[2]);
      pub_rotors_.publish(combined_rotor_command_msg);
    }

    bool MPCAttitudeControllerNode::calculateControlCommand(Eigen::Matrix<double, kInputSize, 1> *control_commands,
                                                            MPCAttitudeController *linear_mpc_commanded_angle) {
      Eigen::Matrix<double, kInputSize, 1> calculated_control_commands;
      (*linear_mpc_commanded_angle).calculateControlCommand(&calculated_control_commands);
      *control_commands = calculated_control_commands;
      return true;
    }

    bool MPCAttitudeControllerNode::calculateCommands() {

        // set the data to the controllers - angles and angular velocities
        linear_mpc_roll_.setAngleState(euler_mv_.x);
        linear_mpc_roll_.setAngularVelocityState(euler_rate_mv_.x);

        linear_mpc_pitch_.setAngleState(euler_mv_.y);
        linear_mpc_pitch_.setAngularVelocityState(euler_rate_mv_.y);

        // calculate the control signals - MAIN ALGORITHM !!!!!
        calculateControlCommand(&roll_commands_, &linear_mpc_roll_);
        calculateControlCommand(&pitch_commands_,&linear_mpc_pitch_);
    }

    void MPCAttitudeControllerNode::publishCommands() {
        assert(pitch_commands_.data());
        assert(roll_commands_.data());

        std_msgs::Float64 mass0_command_msg, mass1_command_msg, mass2_command_msg, mass3_command_msg;
        mass0_command_msg.data =  pitch_commands_[0];
        mass1_command_msg.data = -roll_commands_[0];
        mass2_command_msg.data = -pitch_commands_[1];
        mass3_command_msg.data =  roll_commands_[1];

        // publish the new references for the masses
        pub_mass0_.publish(mass0_command_msg);
        pub_mass1_.publish(mass1_command_msg);
        pub_mass2_.publish(mass2_command_msg);
        pub_mass3_.publish(mass3_command_msg);

        mav_msgs::Actuators rotor_attitude_command_msg;
        rotor_attitude_command_msg.header.stamp = ros::Time::now();
        rotor_attitude_command_msg.angular_velocities.clear();
        rotor_attitude_command_msg.angular_velocities.push_back(+ pitch_commands_[2]);
        rotor_attitude_command_msg.angular_velocities.push_back(+ roll_commands_[3]);
        rotor_attitude_command_msg.angular_velocities.push_back(+ pitch_commands_[3]);
        rotor_attitude_command_msg.angular_velocities.push_back(+ roll_commands_[2]);
        pub_rotors_attitude_.publish(rotor_attitude_command_msg);
    }

    void MPCAttitudeControllerNode::run() {

      // define sampling time
      // needs to be set in the controller as well, change to be unique !!! TODO
      double sampling_time = 0.04;
      ros::Rate loop_rate(1.0 / sampling_time); // 25 Hz -> Ts = 0.04 s

       while (ros::ok()){
           ros::spinOnce();

           // if all the measurements are received
           if (imu_received_ &&
               movable_mass_0_state_received_ && movable_mass_1_state_received_ &&
               movable_mass_2_state_received_ && movable_mass_3_state_received_ &&
               motor_speed_received_) {

               calculateCommands(); // calculate the output
               publishCommands(); // send the received commands to output

               // reset the flags for massages
               imu_received_                  = false;
               movable_mass_0_state_received_ = false;
               movable_mass_1_state_received_ = false;
               movable_mass_2_state_received_ = false;
               movable_mass_3_state_received_ = false;
               motor_speed_received_          = false;
           }

           // go to another anotation and keep the sampling time
           loop_rate.sleep();

      }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_mpc_ctl");

    // fully initialize the node
    ros::NodeHandle nh, private_nh("~");

    mav_control_attitude::MPCAttitudeControllerNode MPC_attitude_controller_node(nh, private_nh);

    // run the regulation
    MPC_attitude_controller_node.run();

    return 0;
}