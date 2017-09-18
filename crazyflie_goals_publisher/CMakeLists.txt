cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_goals_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS
   INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

add_executable(goals_publisher
  src/goals_publisher.cpp
  src/goals_publisher_node.cpp
)

target_link_libraries(goals_publisher
  ${catkin_LIBRARIES}
)
