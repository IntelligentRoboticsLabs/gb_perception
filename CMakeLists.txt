cmake_minimum_required(VERSION 3.5)
project(gb_perception)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Caffe REQUIRED)
find_package(OpenPose REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  rclcpp_cascade_lifecycle
  ament_index_cpp
  sensor_msgs
  cv_bridge
)

include_directories(include
  ${OpenPose_INCLUDE_DIRS}
)
add_library(people_detector SHARED
  src/openpose_detector/OpenPoseDetector.cpp
  src/gb_perception/PeoplePerceptionNode.cpp
)
ament_target_dependencies(people_detector ${dependencies})

add_executable(gb_perception_main
  src/gb_perception_main.cpp
)
ament_target_dependencies(gb_perception_main ${dependencies})
target_link_libraries(gb_perception_main
  people_detector
  ${OpenCV_LIBS}
  ${OpenPose_LIBS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  people_detector
  gb_perception_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(openpose_detector)
ament_export_dependencies(${dependencies})

ament_package()
