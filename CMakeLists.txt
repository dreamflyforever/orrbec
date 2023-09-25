#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(or)

add_executable(or core.cpp)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
set_property(TARGET or PROPERTY CXX_STANDARD 11)
include_directories("/home/jim/workspace/db01/orbbec_sdk")
link_directories("/home/jim/workspace/db01/orbbec_sdk/libobsensor")
link_libraries("/home/jim/workspace/db01/orbbec_sdk/libobsensor/libOrbbecSDK.so")
#find_package(OrbbecSDK)
target_link_libraries(or "/home/jim/workspace/db01/orbbec_sdk/libobsensor/libOrbbecSDK.so" ${OpenCV_LIBS} )
