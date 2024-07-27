cmake_minimum_required(VERSION 2.8.3)
project(odom_converter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_filters
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} 
    src/odom_converter.cpp 
    src/odom_converter_node.cpp 
)
target_link_libraries( ${PROJECT_NAME}
    ${catkin_LIBRARIES} 
)