cmake_minimum_required(VERSION 3.13)
project(SFM)
find_package(OpenCV REQUIRED)
include_directories(include)
set (
src
src/main.cpp
src/PnpSolver.cpp
src/Calibrator.cpp
src/FeaturesPts.cpp)
set(CMAKE_CXX_STANDARD 11)
add_executable(SFM ${src})
target_link_libraries(SFM ${OpenCV_LIBS})