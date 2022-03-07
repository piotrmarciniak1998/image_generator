cmake_minimum_required(VERSION 3.0.2)
project(image_generator)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  sensor_msgs
)

add_service_files(
  FILES
  ImageToSave.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch image_generator
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

