cmake_minimum_required(VERSION 3.14)
project(untitled)

set(CMAKE_CXX_STANDARD 14)
INCLUDE_DIRECTORIES( /usr/local/Cellar/flann/1.9.1_7/include )
LINK_DIRECTORIES(/usr/local/Cellar/flann/1.9.1_7/lib )

find_package(OpenCV REQUIRED)

add_executable(untitled main.cpp viewer.h)

# Thread
find_package(Threads REQUIRED)

# PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
# Additional include directories
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# Preprocessor definitions
add_definitions(${PCL_DEFINITIONS})
# For disabling precompiled features
#add_definitions(-DPCL_NO_PRECOMPILE)
# Additional library directories
link_directories(${PCL_LIBRARY_DIRS})
# Additional dependencies
#target_link_libraries(olp ${PCL_LIBRARIES})

#include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled ${OpenCV_LIBS} ${PCL_LIBRARIES}) # replace binary_name