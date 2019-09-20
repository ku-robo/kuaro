cmake_minimum_required(VERSION 2.8.3)
project(joy_con_core)

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
  ambi_core
  helper_node
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
	joy_con_param
    CATKIN_DEPENDS
  	roscpp
  	rospy
	ambi_core
	helper_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#############################################################################################
## joy_con_param ##

#libraryを新規作成するための処理
add_library(joy_con_param
  src/joy_con_param/joy_con_param.cpp
)

#<$program_name>のプログラムについて，追加するlibを記述する処理.今回なら<$program_name>=<$lib_name>
target_link_libraries(joy_con_param
    ${catkin_LIBRARIES}
)

#buildの順番を明示化．登録するlib(<$lib_name>)より先に，他のものをbuildする
add_dependencies(joy_con_param ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
