Author: David Gitz
Task: Board
Nodes:
A. boardcontroller_node
Usage: This node should be run for every device that has at least 1 Arduino Board installed.
Purpose:
Communicates with ArduinoBoard.  For more info view: 
https://github.com/dgitz/icarus_rover_v2/wiki/ICARUS_ROVER_V2_TASK_BOARDCONTROLLERNODE

Unit Tests:
1. To Run:
    cd ~/catkin_ws
    catkin_make run_tests_icarus_rover_v2_gtest_test_boardcontroller_node_process
2. To View:
    cd ~/catkin_ws
    catkin_test_results build/icarus_rover_v2

Test Results stored at: ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_boardcontroller_node_process.xml




