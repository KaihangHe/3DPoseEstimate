cmake_minimum_required(VERSION 2.8)
project(SFM)
find_package(OpenCV 4.0.1 REQUIRED)
include_directories(include)
set(
        src
        src/main.cpp
        src/Calibrator.cpp
        src/FeaturePtsCatcher.cpp src/CloudPtsGenerator.cpp include/CloudPtsGenerator.h src/GireCamera.cpp include/GireCamera.h src/VizViewer.cpp include/VizViewer.h src/UnitTest.cpp include/UnitTest.h)
set(CMAKE_CXX_STANDARD 11)
add_executable(SFM ${src})
target_link_libraries(SFM ${OpenCV_LIBS})
execute_process(COMMAND echo "by nicapoet")
