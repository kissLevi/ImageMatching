cmake_minimum_required(VERSION 3.14)
project(untitled)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(untitled main.cpp)


include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled ${OpenCV_LIBS}) # replace binary_name