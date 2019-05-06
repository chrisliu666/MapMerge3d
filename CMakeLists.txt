cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(CalculateRotAngle)

find_package(PCL 1.2 REQUIRED)

include_directories("/usr/include/eigen3" ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cal cal.cpp)
target_link_libraries (cal ${PCL_LIBRARIES})
