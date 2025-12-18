cmake_minimum_required(VERSION 3.10)
project(opencv_tracking_standalone)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. OpenCV finden
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV found! Version: ${OpenCV_VERSION}")

# 2. Dateien automatisch sammeln (alle Unterordner von PipelineChuckNorris)
file(GLOB_RECURSE SOURCES
        "PipelineChuckNorris/*.cpp"
        "PipelineChuckNorris/*.h"
)

# 3. Executable NUR EINMAL erstellen
# Falls "driver_cpp_node" schon woanders definiert ist, nenne es hier testweise um,
# z.B. in "lane_detection_node"
add_executable(driver_cpp_node ${SOURCES})

# 4. Include-Pfade setzen
# Das erlaubt #include "A/ColorSpaceScaling.h" statt langer Pfade
target_include_directories(driver_cpp_node PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/PipelineChuckNorris
)

# 5. Bibliotheken verlinken
target_link_libraries(driver_cpp_node
        ${OpenCV_LIBS}
)