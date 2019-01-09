#util
add_library(pose_helper PoseHelper.cpp)
add_dependencies(pose_helper icarus_rover_v2_generate_messages_cpp)
catkin_add_gtest(test_pose_helper unit_tests/test_pose_helper.cpp)
target_link_libraries(test_pose_helper ${catkin_LIBRARIES} ${Boost_LIBRARIES} pose_helper)
