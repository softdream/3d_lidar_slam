cmake_minimum_required(VERSION 2.8)

project(test)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(test main.cpp)
target_link_libraries(test ${OpenCV_LIBS})