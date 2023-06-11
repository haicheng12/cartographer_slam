cmake_minimum_required(VERSION 2.8.3)
project(wit_ros_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_odom 
  src/imu_odom/imu_odom.cpp
  src/imu_odom/imu_odom_core.cpp)
target_link_libraries(imu_odom ${catkin_LIBRARIES})

install(DIRECTORY demo driver robot_para rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

