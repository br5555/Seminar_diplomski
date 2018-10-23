#ifndef PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
#define PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H

#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "rosgraph_msgs/Clock.h"
#include "control_msgs/JointControllerState.h" // for moving masses states
#include <mav_msgs/Actuators.h> // for motor speed states
#include "morus_msgs/CommandMovingMasses.h"
#include "morus_msgs/AngleAndAngularVelocity.h"

#include "morus_control/attitude_mpc_ctl.h"
#include "morus_control/attitude_teleop_joy.h"

// dynamic reconfigure files
#include <dynamic_reconfigure/server.h>
#include <morus_control/MPCAttitudeControllerConfig.h>

namespace mav_control_attitude {

    class MPCAttitudeControllerNode{

    public:
        MPCAttitudeControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
        ~MPCAttitudeControllerNode();
        void run();

    private:
        ros::NodeHandle nh_, private_nh_;

        // classes and structures used
        MPCAttitudeController linear_mpc_roll_, linear_mpc_pitch_;
        AttitudeJoy attitude_joy_;

        // dynamic reconfigure server init
        dynamic_reconfigure::Server<morus_control::MPCAttitudeControllerConfig> dyn_config_server_;
        void DynConfigCallback(morus_control::MPCAttitudeControllerConfig &config, uint32_t level);

        // calculation of the future input signals
        virtual bool calculateControlCommand(Eigen::Matrix<double, kInputSize, 1> *control_commands,
                                             MPCAttitudeController *linear_mpc_commanded_angle);
        bool calculateCommands();

        // publishers
        ros::Publisher pub_mass0_;
        ros::Publisher pub_mass1_;
        ros::Publisher pub_mass2_;
        ros::Publisher pub_mass3_;
        ros::Publisher pub_rotors_;
        ros::Publisher pub_rotors_attitude_;
        // debugging publisher
        ros::Publisher pub_angle_state_;

        void publishCommands();

        // variables to hold the control variable
        Eigen::Matrix<double, kInputSize, 1> roll_commands_;
        Eigen::Matrix<double, kInputSize, 1> pitch_commands_;
        bool start_flag_;

        // subscribers
        ros::Subscriber imu_subscriber_;
            void AhrsCallback(const sensor_msgs::Imu& msg);
            struct euler_mv {
                double x;
                double y;
                double z;
            } euler_mv_;
            struct euler_rate_mv {
                double x;
                double y;
                double z;
            } euler_rate_mv_;
            bool imu_received_; // received msg from imu

        ros::Subscriber mot_vel_ref_subscriber_;
            void MotVelRefCallback(const std_msgs::Float32& msg);
            float w_sp_;

        ros::Subscriber euler_ref_subscriber_;
            void EulerRefCallback(const geometry_msgs::Vector3& msg);
            geometry_msgs::Vector3 euler_sp_;

        ros::Subscriber clock_subscriber_;
            void ClockCallback(const rosgraph_msgs::Clock& msg);
            rosgraph_msgs::Clock clock_read_;

        ros::Subscriber movable_mass_0_state_subscriber_;
            void MovingMass0Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_0_position_;
            double movable_mass_0_speed_;
            bool movable_mass_0_state_received_;

        ros::Subscriber movable_mass_1_state_subscriber_;
            void MovingMass1Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_1_position_;
            double movable_mass_1_speed_;
            bool movable_mass_1_state_received_;

        ros::Subscriber movable_mass_2_state_subscriber_;
            void MovingMass2Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_2_position_;
            double movable_mass_2_speed_;
            bool movable_mass_2_state_received_;

        ros::Subscriber movable_mass_3_state_subscriber_;
            void MovingMass3Callback(const control_msgs::JointControllerState& msg);
            double movable_mass_3_position_;
            double movable_mass_3_speed_;
            bool movable_mass_3_state_received_;

        ros::Subscriber motor_speed_subscriber_;
            void MotorSpeedCallback(const mav_msgs::Actuators& msg);
            double motor_0_speed_;
            double motor_1_speed_;
            double motor_2_speed_;
            double motor_3_speed_;
            bool motor_speed_received_;

        ros::Subscriber motor_speed_height_subscriber_;
            void MotorSpeedHeightCallback(const mav_msgs::Actuators& msg);

        // debug info
        bool verbose_;

        // variable for reference setting
        bool automatic_reference_;
        void setAutomaticReference();
    };
}
#endif //PROJECT_MASS_CTL_ATTITUDE_MPC_NODE_H
