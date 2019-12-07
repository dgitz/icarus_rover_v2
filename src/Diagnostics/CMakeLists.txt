add_library(lcddriver Driver/LCDDriver.cpp)

#Diagnostic Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/diagnostic_node)
add_library(diagnostic_node_process DiagnosticNodeProcess.cpp)
add_dependencies(diagnostic_node_process eros_generate_messages_cpp)
target_link_libraries(diagnostic_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(diagnostic_node diagnostic_node.cpp)
target_link_libraries(diagnostic_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger resourcemonitor diagnostic_node_process lcddriver)
catkin_add_gtest(test_diagnostic_node_process unit_tests/test_diagnostic_node_process.cpp)
target_link_libraries(test_diagnostic_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} diagnostic_node_process logger serialmessage)

#Safety Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/safety_node)
add_library(safety_node_process SafetyNodeProcess.cpp)
add_dependencies(safety_node_process eros_generate_messages_cpp)
target_link_libraries(safety_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    add_executable(safety_node safety_node.cpp) #Only compatible for armv7l
    target_link_libraries(safety_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor safety_node_process terminalhatdriver)
endif ()
catkin_add_gtest(test_safety_node_process unit_tests/test_safety_node_process.cpp)
target_link_libraries(test_safety_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} safety_node_process)

#DataLogger_Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/datalogger_node)
add_library(datalogger_node_process DataLoggerNodeProcess.cpp)
add_dependencies(datalogger_node_process eros_generate_messages_cpp)
target_link_libraries(datalogger_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL x86_64)
    add_executable(datalogger_node datalogger_node.cpp)
    target_link_libraries(datalogger_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger resourcemonitor datalogger_node_process)
endif ()
catkin_add_gtest(test_datalogger_node_process unit_tests/test_datalogger_node_process.cpp)
target_link_libraries(test_datalogger_node_process  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} datalogger_node_process logger)

#Snapshot Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/snapshot_node)
add_library(snapshot_node_process SnapshotNodeProcess.cpp)
add_dependencies(snapshot_node_process eros_generate_messages_cpp)
target_link_libraries(snapshot_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} )
add_executable(snapshot_node snapshot_node.cpp)
target_link_libraries(snapshot_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger resourcemonitor snapshot_node_process)
catkin_add_gtest(test_snapshot_node_process unit_tests/test_snapshot_node_process.cpp)
target_link_libraries(test_snapshot_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} snapshot_node_process logger)


