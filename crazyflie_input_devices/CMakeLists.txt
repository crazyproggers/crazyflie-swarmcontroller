cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_input_devices)

find_package(catkin REQUIRED COMPONENTS
  std_srvs
  roscpp
  crazyflie_goals_publisher
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  crazyflie_goals_publisher
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard
  src/keyboard.cpp
)

target_link_libraries(keyboard
  ${catkin_LIBRARIES}
)
