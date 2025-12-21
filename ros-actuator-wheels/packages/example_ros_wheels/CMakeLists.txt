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
  ${PROJECT_SOURCE_DIR}/src/core/PipelineChuckNorris
)

add_executable(driver_cpp_node
    src/main.cpp
    src/core/runtime_config.cpp
    src/core/CompareMethod/CompareMethod.cpp

    src/core/PipelineChuckNorris/LineDetectionPipeline.cpp
    src/core/PipelineChuckNorris/A/ColorSpaceScaling.cpp
    src/core/PipelineChuckNorris/B/NoiseReduction.cpp
    src/core/PipelineChuckNorris/C/FuzzyCannyEdgeDetection.cpp
    src/core/PipelineChuckNorris/D/ROISelection.cpp
    src/core/PipelineChuckNorris/E/LineDetection.cpp
    src/core/PipelineChuckNorris/F/Tracking.cpp
)

target_link_libraries(driver_cpp_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
