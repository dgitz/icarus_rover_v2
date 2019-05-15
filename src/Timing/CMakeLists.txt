#TimeMaster Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/timemaster_node)
add_library(timemaster_node_process TimeMasterNodeProcess.cpp)
target_link_libraries(timemaster_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(timemaster_node timemaster_node.cpp)
target_link_libraries(timemaster_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor timemaster_node_process)
catkin_add_gtest(test_timemaster_node_process unit_tests/test_timemaster_node_process.cpp)
target_link_libraries(test_timemaster_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} timemaster_node_process logger serialmessage)


#TimeSlave Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/timeslave_node)
add_library(timeslave_node_process TimeSlaveNodeProcess.cpp)
target_link_libraries(timeslave_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(timeslave_node timeslave_node.cpp)
target_link_libraries(timeslave_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor timeslave_node_process)
catkin_add_gtest(test_timeslave_node_process unit_tests/test_timeslave_node_process.cpp)
target_link_libraries(test_timeslave_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} timeslave_node_process)
