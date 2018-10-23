cmake_minimum_required(VERSION 2.8.3)
project(morus_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    rosgraph_msgs
    genmsg
    control_msgs
    sensor_msgs
    morus_msgs
    dynamic_reconfigure
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(cmake_modules REQUIRED)

# enable the use of Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} )

# dynamic reconfigurer
generate_dynamic_reconfigure_options(config/LinearMPC.cfg)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

catkin_package(
    INCLUDE_DIRS include lib src config
    LIBRARIES morus_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  lib
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
# LIBRARIES #
#############

# convex solver - CVXGEN
add_library(cvxgen_solver_moving_masses
        lib/ldl.c
        lib/solver.c
        lib/matrix_support.c
        lib/util.c
        lib/variables_definition.c
)

# general algorithm for executing
add_library(attitude_ctl_mpc
        src/attitude_mpc_ctl.cpp
        src/steady_state_calculation.cpp
        src/KFDisturbanceObserver.cpp
        lib/solver.c
)

add_dependencies(attitude_ctl_mpc
        cvxgen_solver_moving_masses
)

target_link_libraries(attitude_ctl_mpc
        cvxgen_solver_moving_masses
        ${catkin_LIBRARIES}
)

add_library(attitude_teleop_joy
        src/attitude_teleop_joy.cpp
        include/morus_control/attitude_teleop_joy.h)

add_executable(attitude_mpc_ctl_node
        src/nodes/attitude_mpc_ctl_node.cpp
        )

add_dependencies(attitude_mpc_ctl_node
        ${PROJECT_NAME}_gencfg)

target_link_libraries(attitude_mpc_ctl_node
        ${catkin_LIBRARIES}
        attitude_ctl_mpc
        cvxgen_solver_moving_masses
        attitude_teleop_joy)