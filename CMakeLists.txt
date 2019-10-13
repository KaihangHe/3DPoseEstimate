cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(3DPoseEstimate)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBS_DIRS})
add_definitions(${PCL_DEFINTIONS})
link_libraries(/lib/libMVSDK.so)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(SRC_CODE
        src/main.cpp
        src/Calibrator.cpp
        src/FeaturePtsCatcher.cpp
        src/CloudPtsGenerator.cpp
        src/VizViewer.cpp
        src/MindVisionCamera.cpp
        src/PoseEstiamte.cpp)

# Declare the executable target
add_executable(3DPoseEstimate ${SRC_CODE})

# Link application with OpenCV libraries
target_link_libraries(3DPoseEstimate ${OpenCV_LIBS} ${PCL_LIBRARIES})