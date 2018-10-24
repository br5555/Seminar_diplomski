cmake_minimum_required(VERSION 2.8.3)
project(ceres_mpc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package( OpenCV  REQUIRED COMPONENTS core highgui flann calib3d )

 FIND_PACKAGE(Eigen3 REQUIRED)
 include_directories(${EIGEN3_INCLUDE_DIR})

# ceres solver from google
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})



SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES edge_alignment
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_mm_ceres
    src/mpc_mm_ceres.cpp
)

add_executable(ceres_mpc
    src/ceres_mpc.cpp
)

add_executable(ceres_mpc_2
    src/ceres_mpc_2.cpp
)

add_executable(ceres_mpc_3
    src/ceres_mpc3.cpp
)

target_link_libraries(mpc_mm_ceres
   ${catkin_LIBRARIES}
   ${CERES_LIBRARIES}
)

target_link_libraries(ceres_mpc
   ${catkin_LIBRARIES}
   ${CERES_LIBRARIES}
)

target_link_libraries(ceres_mpc_2
   ${catkin_LIBRARIES}
   ${CERES_LIBRARIES}
)

target_link_libraries(ceres_mpc_3
   ${catkin_LIBRARIES}
   ${CERES_LIBRARIES}
)

