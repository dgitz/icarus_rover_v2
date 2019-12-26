#Speaker Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/speaker_node)
add_library(speaker_node_process SpeakerNodeProcess.cpp)
add_dependencies(speaker_node_process eros_generate_messages_cpp)
target_link_libraries(speaker_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(speaker_node speaker_node.cpp)
target_link_libraries(speaker_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger resourcemonitor speaker_node_process)
catkin_add_gtest(test_speaker_node_process unit_tests/test_speaker_node_process.cpp)
target_link_libraries(test_speaker_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} speaker_node_process )

#NetworkTransceiver Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/networktransceiver_node)
add_library(networktransceiver_node_process NetworkTransceiverNodeProcess.cpp)
add_dependencies(networktransceiver_node_process eros_generate_messages_cpp)
target_link_libraries(networktransceiver_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(networktransceiver_node networktransceiver_node.cpp)
target_link_libraries(networktransceiver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger udpmessage resourcemonitor networktransceiver_node_process)
catkin_add_gtest(test_networktransceiver_node_process unit_tests/test_networktransceiver_node_process.cpp)
target_link_libraries(test_networktransceiver_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} networktransceiver_node_process logger serialmessage)

#Topic Remapper Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/topicremapper_node)
add_library(topicremapper_node_process TopicRemapperNodeProcess.cpp)
add_dependencies(topicremapper_node_process eros_generate_messages_cpp)
target_link_libraries(topicremapper_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(topicremapper_node topicremapper_node.cpp)
target_link_libraries(topicremapper_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} logger resourcemonitor topicremapper_node_process)
catkin_add_gtest(test_topicremapper_node_process unit_tests/test_topicremapper_node_process.cpp)
target_link_libraries(test_topicremapper_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} topicremapper_node_process logger)

#Database Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/database_node)
add_library(database_node_process DatabaseNodeProcess.cpp)
add_dependencies(database_node_process eros_generate_messages_cpp)
target_link_libraries(database_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} sqlite3 )
add_executable(database_node database_node.cpp)
target_link_libraries(database_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} sqlite3 logger resourcemonitor database_node_process)
catkin_add_gtest(test_database_node_process unit_tests/test_database_node_process.cpp)
target_link_libraries(test_database_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} database_node_process logger)
