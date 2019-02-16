cmake_minimum_required(VERSION 3.9)
project(callibration)
set( OpenCV_DIR /home/nagesh/opencv-2.4.10/release/)
set( OpenCV_FOUND TRUE)

find_package(OpenCV 2.4 REQUIRED )

#include_directories("/usr/local/include/")

set(CMAKE_CXX_STANDARD 11)

add_executable(track track.cpp)
target_link_libraries(track -L/usr/lib -L/usr/local/lib -std=c++11 ${OpenCV_LIBS})
add_executable(camera_callib main.cpp)
target_link_libraries(camera_callib -L/usr/lib -L/usr/local/lib -std=c++11 ${OpenCV_LIBS})

