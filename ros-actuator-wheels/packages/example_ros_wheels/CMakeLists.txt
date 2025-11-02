cmake_minimum_required(VERSION 2.8.3)
project(example_ros_wheels)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver_cpp_node src/main.cpp)

target_link_libraries(driver_cpp_node
  ${catkin_LIBRARIES}
)

#add_executable(test_node src/test_lanes.cpp src/LaneDetector.cpp)
#target_link_libraries(test_node ${OpenCV_LIBRARIES})
