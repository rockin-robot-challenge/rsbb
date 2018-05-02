cmake_minimum_required(VERSION 2.8.3)
project(rsbb_bmbox)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  message_runtime
  rsbb_benchmarking_messages
  std_msgs
  tf
  geometry_msgs
  roslaunch
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
##add_message_files(
#  FILES
#  BmBoxState.msg
#  ClientState.msg
#  RefBoxState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  rsbb_benchmarking_messages
#  std_msgs
#  geometry_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/bmbox_test
  scripts/refbox_test
  scripts/client_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#roslaunch_add_file_check(launch)

