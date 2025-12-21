cmake_minimum_required(VERSION 2.8.3)
project(example_ros_wheels)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src/core
)

add_executable(driver_cpp_node
    src/main.cpp
    src/core/runtime_config.cpp
    src/core/CompareMethod/CompareMethod.cpp
)

target_link_libraries(driver_cpp_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
