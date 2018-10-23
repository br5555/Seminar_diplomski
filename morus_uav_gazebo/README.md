# morus_uav_gazebo
Repository for developing UAV simulator based on Gazebo.

## Instalation guides 
1) ROS Kinetic, Ubuntu 16.04., best to install ros-full-desktop
2) For changing the contents: fork the github package "https://github.com/larics/morus_uav_gazebo", clone in own ros workspace and turn to "kinetic-dev" branch
3) Clone repo "https://github.com/google/glog"
	git checkout 942b8dfbe6bd567b1a0b41531016fd6cf10f0b3c
4) Clone repo "https://github.com/ethz-asl/mav_comm"
	git checkout de1b6294fa30f2c5fb892831bc86bd7ec8c08d00
5) Clone repo "https://github.com/ethz-asl/rotors_simulator" to ros workspace, follow instalation guide
	git checkout 97b3da2d02ab498b0c9d7a15d0297e72fe6b6482
6) Clone repo "https://github.com/larics/morus_uav_ros_msgs" to ros workspace

## Running the MPC simulation
```sh
  $ roscore &
  $ roslaunch morus_gazebo morus_multirotor_empty_world_ctl_mpc.launch MPC_control_run:=true
```

* With this launch file, following will be ran: 
    * _Gazebo_ simulation, 
    * height controller (PID), 
    * attitude controller (MPC) 
    * _rqt_gui_ for posting the references

In _rqt_gui_ post desired height and angles on topics:
* "/morus/euler_ref" (angles for roll-x and pitch-y)
* "/morus/pos_ref" (reference for height-z)

After the desired values are set, run:
```sh
  $ rosservice call gazebo/unpause_physics
```
to start the simulation.

## Saving plots
There is a save file called _process_bag.sh_, position in the required directory and call
```sh
  $ ./process_bag.sh "name_of_the_directory_to_save"
```

inside the .sh script define the topics wanted to save and plot later in Matlab or desired software

## Changing the optimization problem
Go to CVXGEN software webpage (https://cvxgen.com/docs/index.html) and generate new files for your optimization problem:
* _solve.h_,
* _ldl.c_, 
* _matrix_support.c_, 
* _solver.c_ 
* _util.c_

Replace the desired files found in "morus_control/include/morus_control/solve.h" and "morus_control/lib/*" with generated ones.

For c++ compiler compiler support add the following lines at hte beggining:
```sh
 #define SOLVER_H

 // enable the C++ compiler to compile C code
 #ifdef __cplusplus
 extern "C" {
 #endif

 /* Uncomment the next line to remove all library dependencies. */
```
and at the end:
```sh
// enable the C++ compiler to compile C code
#ifdef __cplusplus
}
#endif
```

## Troubleshooting
If the _gazebo_gui_ fails to launch, run in another terminal:
```sh
  $ gzclient
```

If the _gazebo_ fails to launch, run in another terminal:
```sh
  $ killall gazebo gzgui gzserver gzclient
```
and restart the launch file

If nothing else works, run:
```sh
  $ rosclean purge
```
and restart everything.

--------

Contact
-------
Luka Pevec luka.pevec@fer.hr
