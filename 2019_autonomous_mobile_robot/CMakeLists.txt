cmake_minimum_required(VERSION 2.8.3)
project(2019_autonomous_mobile_robot)


## Compile as C++11, supported in ROS Kinetic and newer
# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  cv_bridge
  tf
  laser_geometry
  pcl_conversions
  pcl_ros
)


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system timer thread)


################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
	message_runtime
)



###########
## Build ##
###########


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

################################################
## velodyne_detection ##
add_executable(velodyne_detection
  src/velodyne_detection/main.cpp
  src/velodyne_detection/stdafx.hpp
  src/velodyne_detection/velodyne_detection.cpp
  src/velodyne_detection/velodyne_detection.hpp
)

target_link_libraries(velodyne_detection
	${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)
################################################
## person_manager ##

add_executable(person_manager
  src/person_manager/main.cpp
  src/person_manager/person_manager.cpp
  src/person_manager/person_manager.hpp
  src/person_manager/stdafx.hpp
)

target_link_libraries(person_manager
	${catkin_LIBRARIES}
)

################################################
## waypoint_publisher ##

add_executable(waypoint_publisher
  src/waypoint_publisher/main.cpp
  src/waypoint_publisher/stdafx.hpp
)

target_link_libraries(waypoint_publisher
	${catkin_LIBRARIES}
)

################################################
## person_manager_sim ##

add_executable(person_manager_sim
  src/person_manager_sim/main.cpp
  src/person_manager_sim/person_manager.cpp
  src/person_manager_sim/person_manager.hpp
  src/person_manager_sim/stdafx.hpp
)

target_link_libraries(person_manager_sim
	${catkin_LIBRARIES}
)

################################################
## uway_pointmaker_2019 ##

add_executable(uway_pointmaker_2019
  src/uway_pointmaker/uway_main.cpp
)

target_link_libraries(uway_pointmaker_2019
	${catkin_LIBRARIES}
)

################################################
## waypoint_maker_auto ##

add_executable(waypoint_maker_auto
  src/waypoint_maker_auto/main.cpp
  src/waypoint_maker_auto/stdafx.hpp
)

target_link_libraries(waypoint_maker_auto
	${catkin_LIBRARIES}
)

################################################
## fake_obstacle ##

add_executable(fake_obstacle
	src/fake_obstacle/main.cpp
	src/fake_obstacle/fake_obstacle.cpp
	src/fake_obstacle/fake_obstacle.hpp
)

target_link_libraries(fake_obstacle
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)


#############
## Install ##
#############


#############
## Testing ##
#############
