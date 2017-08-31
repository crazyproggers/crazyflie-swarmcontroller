cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(crazyflie_controller
  src/controller.cpp)

target_link_libraries(crazyflie_controller
  ${catkin_LIBRARIES}
)
