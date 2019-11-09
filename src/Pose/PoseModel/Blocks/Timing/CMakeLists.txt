#Timing Blocks
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/pose_node)
add_library(time_compensate TimeCompensate.cpp)
catkin_add_gtest(test_timing_blocks unit_tests/test_timing_blocks.cpp)
target_link_libraries(test_timing_blocks ${catkin_LIBRARIES} ${Boost_LIBRARIES}   time_compensate create_dummydata)
