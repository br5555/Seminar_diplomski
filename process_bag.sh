# script for saving ROS topics

folder=$1 # define the save folder
mkdir -p $folder  # create a save folder
# mkdir $(date '+%d-%b-%Y') # save the folder with current date

# define the save topics which are goind to be saved

# POSITION AND ORIENTATION OF UAV___________________________________
# references for anlges and z
rostopic echo -p /morus/euler_ref> "$folder/euler_ref.txt" &
rostopic echo -p /morus/pos_ref> "$folder/pose_ref.txt" &

# current state of the UAV - pose
rostopic echo -p /morus/pose> "$folder/pose.txt" &  # px, py, pz
rostopic echo -p /morus/angles> "$folder/angles.txt" & # roll, pitch, jaw
rostopic echo -p /morus/velocity> "$folder/velocity.txt" & # vx, vy, vz
rostopic echo -p /morus/position> "$folder/position.txt" &

#___________________________________________________________________

# MOVING MASSES ____________________________________________________
# command signals 
rostopic echo -p /morus/movable_mass_0_position_controller/command/data> "$folder/mass_0_command.txt" &
rostopic echo -p /morus/movable_mass_1_position_controller/command/data> "$folder/mass_1_command.txt" &
rostopic echo -p /morus/movable_mass_2_position_controller/command/data> "$folder/mass_2_command.txt" & 
rostopic echo -p /morus/movable_mass_3_position_controller/command/data> "$folder/mass_3_command.txt" &

# current states - positions and velocities of masses
rostopic echo -p /morus/movable_mass_0_position_controller/state/process_value> "$folder/mass_0_position.txt" &
rostopic echo -p /morus/movable_mass_0_position_controller/state/process_value_dot> "$folder/mass_0_velocity.txt" & 
rostopic echo -p /morus/movable_mass_1_position_controller/state/process_value> "$folder/mass_1_position.txt" & 
rostopic echo -p /morus/movable_mass_1_position_controller/state/process_value_dot> "$folder/mass_1_velocity.txt" &
rostopic echo -p /morus/movable_mass_2_position_controller/state/process_value> "$folder/mass_2_position.txt" &
rostopic echo -p /morus/movable_mass_2_position_controller/state/process_value_dot> "$folder/mass_2_velocity.txt" &
rostopic echo -p /morus/movable_mass_3_position_controller/state/process_value> "$folder/mass_3_position.txt" &
rostopic echo -p /morus/movable_mass_3_position_controller/state/process_value_dot> "$folder/mass_3_velocity.txt" &
#___________________________________________________________________

# PROPELLERS _______________________________________________________
# command signals from attitude controller
rostopic echo -p /morus/command/attitude/motor_speed> "$folder/motor_speed_attitude_command.txt" &

# command signals from height controller
rostopic echo -p /morus/command/height/motor_speed> "$folder/motor_speed_height_command.txt" &

# command signals combined signal (height + attitude controller)
rostopic echo -p /gazebo/command/motor_speed> "$folder/motor_speed_command.txt" &

# current states of propellers 
rostopic echo -p /morus/motor_speed_lin> "$folder/motor_speed_state.txt" &
#___________________________________________________________________

# MPC DATA _________________________________________________________
rostopic echo -p /morus/mpc/target_states/data> "$folder/target_states.txt" &
rostopic echo -p /morus/mpc/disturbances/data> "$folder/disturbances.txt" &
rostopic echo -p /morus/mpc/target_input/data> "$folder/target_input.txt" &

wait
kill $(jobs -p)
wait
