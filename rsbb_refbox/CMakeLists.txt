cmake_minimum_required(VERSION 2.8.7)
project(roah_rsbb)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rqt_gui
  rqt_gui_cpp
  message_generation
  std_msgs
  std_srvs
  roah_devices
  rsbb_benchmarking_messages
  actionlib_msgs
  actionlib
)

add_definitions("-std=c++0x")
add_definitions("-Wall")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Boost REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp>=0.5.0)
if(YAML_CPP_FOUND)
  include_directories(${YAML_CPP_INCLUDE_DIRS})
  link_directories(${YAML_CPP_LIBRARY_DIRS})
else()
  include_directories("/tmp/yaml-cpp-0.5.1/include")
  set(YAML_CPP_LIBRARIES "/tmp/yaml-cpp-0.5.1/libyaml-cpp.a")
  if(NOT EXISTS ${YAML_CPP_LIBRARIES})
    message(FATAL_ERROR "yaml-cpp>=0.5.0 not found, please run:
    cd /tmp &&
    wget https://yaml-cpp.googlecode.com/files/yaml-cpp-0.5.1.tar.gz &&
    tar xavf yaml-cpp-0.5.1.tar.gz &&
    cd yaml-cpp-0.5.1 &&
    cmake . &&
    make -j8 yaml-cpp")
  endif()
  message(STATUS "  using yaml-cpp from ${YAML_CPP_LIBRARIES}")
endif()

find_package(Qt4 REQUIRED QtCore QtGui)
include(${QT_USE_FILE})


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES ${MESSAGE_FILES})

## Generate services in the 'srv' folder
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
add_service_files(FILES ${SERVICE_FILES}) 

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  roah_devices
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
LIBRARIES rqt_roah_rsbb
CATKIN_DEPENDS message_runtime std_msgs roah_devices rsbb_benchmarking_messages # roscpp
#  DEPENDS system_lib
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(DISAMBIGUATION ROAH_RSBB_)
add_subdirectory(comm/gen/roah_rsbb_msgs/)
add_subdirectory(comm/libs/)
add_subdirectory(comm/include/)

add_subdirectory(gen/)
add_dependencies(roah_rsbb_qt roah_rsbb_generate_messages_cpp)

include_directories(
  ${INCLUDE_DIRS}
)

add_executable(core src/core.cpp)
add_dependencies(core roah_rsbb_generate_messages_cpp)
target_link_libraries(core ${DISAMBIGUATION}roah_rsbb_msgs ${DISAMBIGUATION}protobuf_comm ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(public src/public.cpp)
add_dependencies(public roah_rsbb_generate_messages_cpp)
target_link_libraries(public rqt_roah_rsbb ${catkin_LIBRARIES})

add_executable(shutdown_service src/shutdown_service.cpp)
target_link_libraries(shutdown_service ${catkin_LIBRARIES})

add_executable(sounds src/sounds.cpp)
add_dependencies(sounds roah_rsbb_generate_messages_cpp)
target_link_libraries(sounds ${catkin_LIBRARIES})

file(GLOB RQT_LIB_SOURCES ${PROJECT_SOURCE_DIR}/src/rqt_roah_rsbb/*.cpp)
add_library(rqt_roah_rsbb ${RQT_LIB_SOURCES})
add_dependencies(rqt_roah_rsbb roah_rsbb_generate_messages_cpp)
target_link_libraries(rqt_roah_rsbb roah_rsbb_qt ${QT_LIBRARIES} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB INSTALL_SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
install(PROGRAMS
  ${INSTALL_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS core shutdown_service sounds rqt_roah_rsbb
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roah_rsbb.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
