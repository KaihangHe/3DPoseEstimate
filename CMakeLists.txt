cmake_minimum_required(VERSION 2.8)
project(SFM)
find_package(OpenCV 4.0.1 REQUIRED)
include_directories(include)
link_libraries(/lib/libMVSDK.so)
set(
        src
        src/main.cc
        src/Calibrator.cc
        src/FeaturePtsCatcher.cc
        src/CloudPtsGenerator.cc
        src/VizViewer.cc
        src/UnitTest.cc
        src/MindVisionCamera.cc)
set(CMAKE_CXX_STANDARD 11)
add_executable(SFM ${src})
target_link_libraries(SFM ${OpenCV_LIBS})
execute_process(COMMAND echo "by nicapoet")
