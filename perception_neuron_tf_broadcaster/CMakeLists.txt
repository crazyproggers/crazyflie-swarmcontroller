cmake_minimum_required(VERSION 2.8.3)
project(perception_neuron_tf_broadcaster)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(/usr/include/eigen3)

## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(perception_neuron_tf_broadcaster_node 
	src/perception_neuron_tf_broadcaster_node.cpp
) 

# Specify libraries to link a library or executable target against
 target_link_libraries(perception_neuron_tf_broadcaster_node
   ${catkin_LIBRARIES}
 )
