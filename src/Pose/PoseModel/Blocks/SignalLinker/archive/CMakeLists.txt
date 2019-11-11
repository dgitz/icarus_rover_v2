#SignalLinker Blocks
add_library(linear_acceleration_linker LinearAccelerationLinker.cpp)

catkin_add_gtest(test_signallinker unit_tests/test_signallinker.cpp)
target_link_libraries(test_signallinker ${catkin_LIBRARIES} ${Boost_LIBRARIES} linear_acceleration_linker)
