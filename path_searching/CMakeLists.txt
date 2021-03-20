cmake_minimum_required(VERSION 2.8.3)
project(path_searching)
set (CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

MESSAGE( "PCL_______________________________________ = ${PCL_INCLUDE_DIRS}.")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( path_searching 
    src/kinodynamic_astar.cpp 
    )
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
