cmake_minimum_required (VERSION 2.8)
project (IMUUnitTest)
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)

FIND_PACKAGE (Threads REQUIRED)



file(GLOB SOURCES
	"../Driver/*.h"
	"../Driver/*.cpp"
    "../Driver/110015/*.h"
    "../Driver/110015/*.cpp"
    "test_imu.cpp"
)

include_directories(".")

add_executable(test_imu ${SOURCES})

target_link_libraries(test_imu ${CMAKE_THREAD_LIBS_INIT})
#target_compile_features(NavXTimeStamp PRIVATE cxx_range_for)

