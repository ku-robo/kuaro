cmake_minimum_required(VERSION 2.8.3)
project(func_default)
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
  pluginlib
  ama_core
  ama_struct
)

catkin_package(
   INCLUDE_DIRS
	include
   LIBRARIES
	${PROJECT_NAME}
   CATKIN_DEPENDS
	roscpp
	pluginlib
	ama_core
	ama_struct
)


#このプロジェクトでインクルードすべきものを書く
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



#############################################################################################
## func_default ##

#libraryを新規作成するための処理
add_library(${PROJECT_NAME}
  src/func_default.cpp
)

#<$program_name>のプログラムについて，追加するlibを記述する処理.今回なら<$program_name>=<$lib_name>
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#buildの順番を明示化．登録するlib(<$lib_name>)より先に，他のものをbuildする
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

