cmake_minimum_required(VERSION 2.8.3)
project(2018_tsukuba_challenge)


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
  Mlib
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
## velodyne_detection_2018 ##
add_executable(velodyne_detection_2018
  src/velodyne_detection_2018/main.cpp
  src/velodyne_detection_2018/stdafx.hpp
  src/velodyne_detection_2018/velodyne_detection.cpp
  src/velodyne_detection_2018/velodyne_detection.hpp
)

target_link_libraries(velodyne_detection_2018
	${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)
################################################
## person_manager_2018 ##

add_executable(person_manager_2018
  src/person_manager_2018/main.cpp
  src/person_manager_2018/person_manager.cpp
  src/person_manager_2018/person_manager.hpp
  src/person_manager_2018/stdafx.hpp
)

target_link_libraries(person_manager_2018
	${catkin_LIBRARIES}
)

################################################
## waypoint_publisher_2018 ##

add_executable(waypoint_publisher_2018
  src/waypoint_publisher_2018/main.cpp
  src/waypoint_publisher_2018/stdafx.hpp
)

target_link_libraries(waypoint_publisher_2018
	${catkin_LIBRARIES}
)

################################################
## person_manager_sim_2018 ##

add_executable(person_manager_sim_2018
  src/person_manager_sim_2018/main.cpp
  src/person_manager_sim_2018/person_manager.cpp
  src/person_manager_sim_2018/person_manager.hpp
  src/person_manager_sim_2018/stdafx.hpp
)

target_link_libraries(person_manager_sim_2018
	${catkin_LIBRARIES}
)

################################################
## uway_pointmaker_2018 ##

add_executable(uway_pointmaker_2018
  src/uway_pointmaker_2018/uway_main.cpp
)

target_link_libraries(uway_pointmaker_2018
	${catkin_LIBRARIES}
)

################################################
## waypoint_maker_auto_2018 ##

add_executable(waypoint_maker_auto_2018
  src/waypoint_maker_auto_2018/main.cpp
  src/waypoint_maker_auto_2018/stdafx.hpp
)

target_link_libraries(waypoint_maker_auto_2018
	${catkin_LIBRARIES}
)

################################################
## fake_obstacle ##

add_executable(fake_obstacle_2018
	src/fake_obstacle_2018/main.cpp
	src/fake_obstacle_2018/fake_obstacle.cpp
	src/fake_obstacle_2018/fake_obstacle.hpp
)

target_link_libraries(fake_obstacle_2018
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

################################################
## param_class_2018 ##
add_library(param_class_2018
  src/classes_touse_2018/param_class/param_class.cpp
)
target_link_libraries(param_class_2018
    sub_core_LS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(param_class_2018 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(param_class_node
  src/classes_touse_2018/param_class/main.cpp
)
add_dependencies(param_class_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(param_class_node param_class_2018)

set_target_properties(param_class_node PROPERTIES OUTPUT_NAME param_class_2018)

################################################
## scan_class_2018 ##

add_library(scan_class_2018
  src/classes_touse_2018/scan_class/scan_class.cpp
)
target_link_libraries(scan_class_2018
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(scan_class_2018 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scan_class_node
  src/classes_touse_2018/scan_class/main.cpp
)
add_dependencies(scan_class_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scan_class_node
    scan_class_2018
    ${catkin_LIBRARIES}
)

set_target_properties(scan_class_node PROPERTIES OUTPUT_NAME scan_class_2018)


################################################
## abort_move ##

add_library(abort_move
  src/classes_touse_2018/abort_move/abort_move.cpp
)
target_link_libraries(abort_move
    scan_class_2018
    param_class_2018
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(abort_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(abort_move_node
  src/classes_touse_2018/abort_move/main.cpp
)
add_dependencies(abort_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(abort_move_node
    abort_move
    ${catkin_LIBRARIES}
)

set_target_properties(abort_move_node PROPERTIES OUTPUT_NAME abort_move)


################################################
## underD_person_manager_2018_byF ##

add_library(underD_person_manager_2018_byF
  src/under_development_Fukuda/person_manager_2018_byF/Robot_goals_decider.cpp
)
target_link_libraries(underD_person_manager_2018_byF
    abort_move
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(underD_person_manager_2018_byF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(underD_person_manager_2018_byF_node
  src/under_development_Fukuda/person_manager_2018_byF/main.cpp
)
add_dependencies(underD_person_manager_2018_byF_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(underD_person_manager_2018_byF_node underD_person_manager_2018_byF)

set_target_properties(underD_person_manager_2018_byF_node PROPERTIES OUTPUT_NAME underD_person_manager_2018_byF)

################################################



########## under_development ###################
################################################

################################################


#############
## Install ##
#############


#############
## Testing ##
#############
