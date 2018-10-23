#include <morus_control/attitude_teleop_joy.h>

namespace mav_control_attitude {

  AttitudeJoy::AttitudeJoy(const ros::NodeHandle &nh,
                           const ros::NodeHandle &private_nh)
            :nh_(nh),
             private_nh_(private_nh)
  {
    angle_amplitude_max_ = 0.2;

    euler_ref_pub_ = nh_.advertise<geometry_msgs::Vector3>("euler_ref", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &AttitudeJoy::joyCallback, this);
  }

  AttitudeJoy::~AttitudeJoy()
  {
  }

  void AttitudeJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    geometry_msgs::Vector3 euler_ref_msg;  // prepare the msg for sending
    euler_ref_msg.x = -angle_amplitude_max_*joy->axes[0];
    euler_ref_msg.y =  angle_amplitude_max_*joy->axes[1];
    euler_ref_msg.z =                                0.0;
    euler_ref_pub_.publish(euler_ref_msg);
  }
}
