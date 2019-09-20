cmake_minimum_required(VERSION 2.8.3)
project(ama_struct)

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


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  move_base_msgs
  geometry_msgs
  tf
)

catkin_package(
   INCLUDE_DIRS
	include
   LIBRARIES
	get_point
   CATKIN_DEPENDS
	roscpp
	move_base_msgs
	geometry_msgs
	tf
)

#このプロジェクトでインクルードすべきものを書く
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############################################################################################
## get_point ##

#libraryを新規作成するための処理
add_library(get_point
  src/get_point/get_point.cpp
)

#<$program_name>のプログラムについて，追加するlibを記述する処理.今回なら<$program_name>=<$lib_name>
target_link_libraries(get_point
    ${catkin_LIBRARIES}
)

#buildの順番を明示化．登録するlib(<$lib_name>)より先に，他のものをbuildする
add_dependencies(get_point ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
