#Pose Blocks
add_library(poseacceleration_block PoseAccelerationBlock.cpp)


catkin_add_gtest(test_pose_blocks unit_tests/test_pose_blocks.cpp)
target_link_libraries(test_pose_blocks ${catkin_LIBRARIES} ${Boost_LIBRARIES} basic_kalmanfilter  poseacceleration_block)
