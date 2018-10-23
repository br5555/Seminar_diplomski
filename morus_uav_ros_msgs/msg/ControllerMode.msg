#
# Add comment.
#

uint8 armed                          # values ARMED or DISARMED

uint8 controller_x                   # values AUTO or MAN
uint8 controller_y                   # values AUTO or MAN
uint8 controller_z                   # values AUTO or MAN

uint8 controller_velocity_x          # values AUTO or MAN
uint8 controller_velocity_y          # values AUTO or MAN
uint8 controller_velocity_z          # values AUTO or MAN

uint8 controller_yaw                 # values AUTO or MAN
uint8 controller_yaw_rate            # values AUTO or MAN

morus_uav_ros_msgs/GmIgnition ignition     # ignition set point