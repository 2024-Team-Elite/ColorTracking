cmake_minimum_required(VERSION 3.25)
project( Testing )

include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)

set(CMAKE_CXX_FLAGS_DEBUG, "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Ofast -DNDEBUG -s")


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package( OpenCV REQUIRED )
find_package(depthai CONFIG REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( latency latency.cpp  )
if( supported AND CMAKE_BUILD_TYPE STREQUAL "Release" )
    message(STATUS "IPO / LTO enabled")
    set_property(TARGET latency PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(STATUS "IPO / LTO not enabled")
endif()
target_link_libraries( latency ${OpenCV_LIBS} )
target_link_libraries( latency depthai::opencv)


# if( supported )
#     message(STATUS "IPO / LTO enabled")
#     set_property(TARGET latency PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
# else()
#     message(STATUS "IPO / LTO not supported: <${error}>")
# endif()