cmake_minimum_required(VERSION 2.8.3)
project(kuaro_mover)
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


#作ったプロジェクトに必要なpkgを書いておく
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pluginlib
  ambi_core
  move_core
)

catkin_package(
   INCLUDE_DIRS
	include
   LIBRARIES
	kuaro_mover
   CATKIN_DEPENDS
	roscpp
	std_msgs
	pluginlib
	ambi_core
	move_core
)


#このプロジェクトでインクルードすべきものを書く
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)



#############################################################################################
## kuaro_mover ##

#libraryを新規作成するための処理
add_library(kuaro_mover
  src/kuaro_mover.cpp
)

#<$program_name>のプログラムについて，追加するlibを記述する処理.今回なら<$program_name>=<$lib_name>
target_link_libraries(kuaro_mover
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#buildの順番を明示化．登録するlib(<$lib_name>)より先に，他のものをbuildする
add_dependencies(kuaro_mover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



#libを活用したmainプログラム
add_executable(kuaro_mover_node
  src/kuaro_mover_node.cpp	
)

add_dependencies(kuaro_mover_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#mainプログラムで使うライブラリを引っ張ってくる
target_link_libraries(kuaro_mover_node
    kuaro_mover
)

#一種のオプション．これの場合は，登録したlib(<$lib_name>)の名前で実行すれば，<$program_name>が呼ばれる設定．つまり，<$lib_name>のテストを<$program_name>で行う形
set_target_properties(kuaro_mover_node  PROPERTIES OUTPUT_NAME kuaro_mover)
