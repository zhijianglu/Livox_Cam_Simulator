cmake_minimum_required(VERSION 3.0.2)
project(velodyne_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gazebo_ros_velodyne_laser
 CATKIN_DEPENDS gazebo_ros roscpp sensor_msgs tf
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_ros_velodyne_laser src/gazebo_ros_velodyne_laser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)
