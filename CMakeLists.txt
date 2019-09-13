cmake_minimum_required(VERSION 2.8.3)
project(gb_perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  bica_planning
  bica_graph
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES barcode_reader_nodelet
  CATKIN_DEPENDS roscpp
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qr_perception src/qr_perception.cpp)
target_link_libraries(qr_perception ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
