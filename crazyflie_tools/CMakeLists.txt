cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_tools)

find_package(catkin REQUIRED COMPONENTS
  crazyflie_cpp
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    crazyflie_cpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(scan
  src/scan.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(scan
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### listParams
add_executable(listParams
  src/listParams.cpp
)
target_link_libraries(listParams
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### listLogVariables
add_executable(listLogVariables
  src/listLogVariables.cpp
)
target_link_libraries(listLogVariables
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### reboot
add_executable(reboot
  src/reboot.cpp
)
target_link_libraries(reboot
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
