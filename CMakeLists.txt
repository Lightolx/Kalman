cmake_minimum_required(VERSION 3.14)
project(Kalman)

set(CMAKE_CXX_STANDARD 14)

add_executable(Kalman main.cpp KalmanFilter.cpp KalmanFilter.h)
add_executable(GenePM GenerateProbMap.cpp)