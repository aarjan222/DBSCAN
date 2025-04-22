cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(DBSCAN)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CMAKE_CXX_FLAGS} -pthread -pg")

find_package(PCL 1.11.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(dbscan main.cpp dbscan.cpp)
target_link_libraries(dbscan 
  ${PCL_LIBRARIES}
)