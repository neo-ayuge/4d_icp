cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(4d_icp_demo)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_LIBS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(
  ${PCL_DEFINITIONS}
  -std=c++11
  -g
)

##PCL&c++11
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

add_executable(4dicp src/main.cpp src/registration.cpp)
target_link_libraries(4dicp ${PCL_LIBRARIES} ${OpenCV_LIBS})
