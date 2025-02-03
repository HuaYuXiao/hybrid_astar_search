cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_search)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
	   sensor_msgs
	   geometry_msgs
	   nav_msgs
	   pcl_ros
	   easondrone_msgs
	)

catkin_package(
	INCLUDE_DIRS include
	#  DEPENDS system_lib
)

include_directories(
       SYSTEM
       include
       ${catkin_INCLUDE_DIRS}
       ${Eigen3_INCLUDE_DIRS}
       ${PCL_INCLUDE_DIRS}
     )
link_directories(${PCL_LIBRARY_DIRS})

add_executable(hybrid_astar_search
	src/global_planner_node.cpp
	src/global_planner.cpp
	src/kinodynamic_astar.cpp
	src/occupy_map.cpp
	)

target_link_libraries(hybrid_astar_search
       ${catkin_LIBRARIES}
	   ${PCL_LIBRARIES}
	)
