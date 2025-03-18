cmake_minimum_required(VERSION 2.8)
project( Colortracking )
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package( OpenCV REQUIRED )
find_package(depthai CONFIG REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(catkin REQUIRED COMPONENTS serial)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# include_directories( ${DepthAI_INCLUDE_DIRS} )
# add_executable( Colortracking colorPicker.cpp )
add_executable( Colortracking testing.cpp )

# add_executable( cam_ctrl rgb_camera_control.cpp)
target_link_libraries( Colortracking ${OpenCV_LIBS} )
target_link_libraries( Colortracking PRIVATE depthai::opencv)
target_link_libraries( Colortracking PRIVATE jsoncpp_lib) # Install: sudo apt-get install libjsoncpp-dev

set(CMAKE_CXX_FLAGS "-Wall -Wextra -march=native -Wunused-parameter -Wunused-function")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


# target_link_libraries( Colortracking serial)

# target_link_libraries( cam_ctrl ${DepthAI_LIBS} )
