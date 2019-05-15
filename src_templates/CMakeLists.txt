#Sample_Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/sample_node)
add_library(sample_node_process SampleNodeProcess.cpp)
target_link_libraries(sample_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(sample_node sample_node.cpp)
target_link_libraries(sample_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor sample_node_process)
catkin_add_gtest(test_sample_node_process unit_tests/test_sample_node_process.cpp)
target_link_libraries(test_sample_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} sample_node_process)


