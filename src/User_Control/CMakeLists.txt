#Audio Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/audio_node)
add_library(audio_node_process AudioNodeProcess.cpp)
add_dependencies(audio_node_process eros_generate_messages_cpp)
target_link_libraries(audio_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES})
add_executable(audio_node audio_node.cpp) 
target_link_libraries(audio_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor audio_node_process)
catkin_add_gtest(test_audio_node_process unit_tests/test_audio_node_process.cpp)
target_link_libraries(test_audio_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} audio_node_process)

