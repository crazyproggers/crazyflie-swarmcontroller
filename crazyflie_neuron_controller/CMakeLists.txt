cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_neuron_controller)

find_package(catkin REQUIRED COMPONENTS
  std_srvs
  roscpp
  crazyflie_goals_publisher
  tf
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
  include/
)

add_executable(neuron_controller_node
  src/bone.cpp
  src/hand.cpp
  src/gesture.cpp
  src/gesture_recognizer.cpp
  src/neuron_controller_node.cpp
)

target_link_libraries(neuron_controller_node
  ${catkin_LIBRARIES}
)