cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_cpp)

find_package(catkin)
find_library(USB_LIB usb-1.0)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES crazyflie_cpp
  CATKIN_DEPENDS
  DEPENDS ${USB_LIB}
)

###########
## Build ##
###########

## Additional include folders
include_directories(
  include/crazyflie_cpp
)

## Declare a cpp library
add_library(crazyflie_cpp
  src/Crazyradio.cpp
  src/Crazyflie.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(crazyflie_cpp
  ${USB_LIB}
)