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

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ImageToSave.srv
)
## Generate added messages and services with any dependencies listed here
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

install(DIRECTORY launch image_generator param rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

