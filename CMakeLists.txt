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
# Wir fügen "." hinzu, damit relative Includes funktionieren.
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        .
)

# 3. Quellcode-Dateien definieren (angepasst an neue Struktur)
set(SOURCES
        # Die Main-Datei
        StandardMethodPipeline/main/main.cpp

        # A: Color Space Scaling
        StandardMethodPipeline/A/ColorSpaceScaling.cpp
        StandardMethodPipeline/A/ColorSpaceScaling.h

        # B: Noise Reduction
        StandardMethodPipeline/B/NoiseReduction.cpp
        StandardMethodPipeline/B/NoiseReduction.h

        # C: Fuzzy Canny (Kanten)
        StandardMethodPipeline/C/FuzzyCannyEdgeDetection.cpp
        StandardMethodPipeline/C/FuzzyCannyEdgeDetection.h

        # D: ROI Selection (Zuschneiden)
        StandardMethodPipeline/D/RoiSelection.cpp
        StandardMethodPipeline/D/RoiSelection.h

        # E: Line Detection (Hough) - NEU
        StandardMethodPipeline/E/LineDetection.cpp
        StandardMethodPipeline/E/LineDetection.h

        # F: Tracking (Kalman) - NEU
        StandardMethodPipeline/F/Tracking.cpp
        StandardMethodPipeline/F/Tracking.h

        # Falls du DisplayFourPictures noch irgendwo nutzt, kannst du es einkommentieren:
        # StandardMethodPipeline/FOURPICTURESDISPLAY/DisplayFourPictures.cpp
        # StandardMethodPipeline/FOURPICTURESDISPLAY/DisplayFourPictures.h
)

# 4. Executable erstellen
add_executable(driver_cpp_node ${SOURCES})

# 5. Bibliotheken verlinken
target_link_libraries(driver_cpp_node
        ${OpenCV_LIBS}
)