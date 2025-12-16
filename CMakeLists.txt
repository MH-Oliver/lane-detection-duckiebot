# FIX FÜR DIE WARNUNG: Wir setzen die Version hoch auf 3.10 (Standard heute)
cmake_minimum_required(VERSION 3.10)

project(opencv_tracking_standalone)

# C++ Standard festlegen
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. OpenCV finden
find_package(OpenCV REQUIRED)

# Debug-Ausgabe: Zeigt dir im Terminal, wo er OpenCV gefunden hat
message(STATUS "OpenCV found! Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Include Dirs: ${OpenCV_INCLUDE_DIRS}")

# 2. Include-Verzeichnisse
# FIX FÜR DEN FEHLER: ${OpenCV_INCLUDE_DIRS} fügt "/usr/include/opencv4" hinzu.
# Damit findet er "opencv2/opencv_modules.hpp".
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    .
)

# 3. Quellcode-Dateien definieren
set(SOURCES 
    mainGraf.cpp
        FuzzyCannyEdgeDetection.cpp
        FuzzyCannyEdgeDetection.h
    # main.cpp # auskommentiert
    COLORSPACESCALING/ColorSpaceScaling.cpp
    COLORSPACESCALING/ColorSpaceScaling.h
    NOISEREDUCTION/NoiseReduction.cpp
    NOISEREDUCTION/NoiseReduction.h
    FOURPICTURESDISPLAY/DisplayFourPictures.cpp
    FOURPICTURESDISPLAY/DisplayFourPictures.h
    TRACKING/Tracking.cpp
    TRACKING/Tracking.h
        ROISELECTION/RoiSelection.h
        ROISELECTION/RoiSelection.cpp
        LineDetection/LineDetection.cpp
        LineDetection/LineDetection.h
)

# 4. Executable erstellen
add_executable(driver_cpp_node ${SOURCES})

# 5. Bibliotheken verlinken
target_link_libraries(driver_cpp_node
    ${OpenCV_LIBS}
)