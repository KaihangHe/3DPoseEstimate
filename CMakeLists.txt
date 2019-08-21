cmake_minimum_required(VERSION 2.8)
project(SFM)
find_package(OpenCV 4.0.1 REQUIRED)
include_directories(include)
link_libraries(/lib/libMVSDK.so)
set(
        src
        src/main.cpp
        src/Calibrator.cc
        src/VizViewer.cc
        src/PoseEstiamte.cc
        src/MindVisionCamera.cc
        src/FeaturePtsCatcher.cpp
        src/CloudPtsGenerator.cpp)
set(CMAKE_CXX_STANDARD 11)
add_executable(SFM ${src})
target_link_libraries(SFM ${OpenCV_LIBS})
execute_process(COMMAND echo "by nicapoet")
