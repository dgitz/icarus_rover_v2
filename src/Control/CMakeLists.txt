#Pose Node
#CommandLauncher Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/commandlauncher_node)
add_library(commandlauncher_node_process CommandLauncherNodeProcess.cpp)
add_executable(commandlauncher_node commandlauncher_node.cpp)
target_link_libraries(commandlauncher_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor commandlauncher_node_process)
catkin_add_gtest(test_commandlauncher_node_process unit_tests/test_commandlauncher_node_process.cpp)
target_link_libraries(test_commandlauncher_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} commandlauncher_node_process)

#Command Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/command_node)
add_library(command_node_process CommandNodeProcess.cpp)
add_executable(command_node command_node.cpp)
target_link_libraries(command_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor command_node_process)
catkin_add_gtest(test_command_node_process unit_tests/test_command_node_process.cpp)
target_link_libraries(test_command_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} command_node_process logger serialmessage)

#Master Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/master_node)
add_library(master_node_process MasterNodeProcess.cpp)
add_executable(master_node master_node.cpp)
target_link_libraries(master_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor serialmessage master_node_process)
catkin_add_gtest(test_master_node_process unit_tests/test_master_node_process.cpp)
target_link_libraries(test_master_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} master_node_process logger serialmessage)

#Navigation Node
file(MAKE_DIRECTORY /home/robot/catkin_ws/build/test_results/icarus_rover_v2/navigation_node)
add_library(navigation_node_process NavigationNodeProcess.cpp)
add_executable(navigation_node navigation_node.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} logger resourcemonitor serialmessage navigation_node_process)
catkin_add_gtest(test_navigation_node_process unit_tests/test_navigation_node_process.cpp)
target_link_libraries(test_navigation_node_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} navigation_node_process logger serialmessage)


