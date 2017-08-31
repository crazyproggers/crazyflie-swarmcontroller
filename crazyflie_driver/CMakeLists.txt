cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_driver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  tf
  crazyflie_cpp
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  AddCrazyflie.srv
  RemoveCrazyflie.srv
  UpdateParams.srv
)

add_message_files(
  FILES
  LogBlock.msg
  GenericLogData.msg
)

generate_messages(
  DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    tf
    crazyflie_cpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(crazyflie_server
  src/crazyflie_server.cpp
)

add_dependencies(crazyflie_server
  crazyflie_driver_generate_messages_cpp
)

target_link_libraries(crazyflie_server
  ${catkin_LIBRARIES}
)

add_executable(crazyflie_add_group
  src/crazyflie_add_group.cpp
)

add_dependencies(crazyflie_add_group
  crazyflie_driver_generate_messages_cpp
)

target_link_libraries(crazyflie_add_group
  ${catkin_LIBRARIES}
)