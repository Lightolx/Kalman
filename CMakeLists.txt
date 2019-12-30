cmake_minimum_required(VERSION 3.14)
project(Kalman)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

add_executable(Kalman Simulate.cpp KalmanFilter.cpp KalmanFilter.h)
target_link_libraries(Kalman ${GLOG_LIBRARY})

add_executable(WO WO.cpp)
target_link_libraries(WO ${GLOG_LIBRARY})