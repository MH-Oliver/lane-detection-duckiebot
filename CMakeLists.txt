cmake_minimum_required(VERSION 2.8.3)
project(example_ros_wheels)

# KORRIGIERTER BLOCK
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs  # Das "Wörterbuch", das wir in Problem 1 geholt haben
  cv_bridge        # Das ist die "Brücke" von ROS zu OpenCV
)

# Füge diesen Befehl HINZU. Er findet OpenCV, das von cv_bridge bereitgestellt wird.
find_package(OpenCV REQUIRED)
set(SOURCES 
        src/mainGraf.cpp
        #src/main.cpp ein usn auskommentieren wenn gewollt
        src/COLORSPACESCALING/ColorSpaceScaling.cpp
        src/COLORSPACESCALING/ColorSpaceScaling.h
        src/NOISEREDUCTION/NoiseReduction.cpp
        src/NOISEREDUCTION/NoiseReduction.h
        src/FOURPICTURESDISPLAY/DisplayFourPictures.cpp
        src/FOURPICTURESDISPLAY/DisplayFourPictures.h
        src/TRACKING/Tracking.cpp
        src/TRACKING/Tracking.h

        src/ROISELECTION/RoiSelection.cpp
        src/ROISELECTION/RoiSelection.h
        )
        
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

# 2. ZUERST das Executable erstellen
add_executable(driver_cpp_node ${SOURCES})

# 3. DANN die Bibliotheken verlinken
# ${catkin_LIBRARIES} enthält bereits OpenCV, da es oben in COMPONENTS steht.
target_link_libraries(driver_cpp_node
        ${catkin_LIBRARIES}
)
