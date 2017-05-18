Author: David Gitz
Task: Board
Nodes:
A. hatcontroller_node
Usage: This node should be run on every ControlModule (Raspberry Pi).
Purpose: Controls all directly connected Hats/Capes.
Unit Tests:
1.  Process Unit Tests:
2.  Hat Unit Tests:
  Compile: 
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Board/unit_tests/
  >>g++ -lm -lwiringPi ../ServoHatDriver.cpp test_servohat.cpp -o test_servohat
    

B. gpio_node
Usage: This node should be run for every device that has at least 1 GPIO Board installed.
Purpose:
Communicates with GPIO_Board.  For more info view: 
https://github.com/dgitz/icarus_rover_v2/wiki/ICARUS_ROVER_V2_TASK_GPIONODE

Unit Tests:
1. To Run:
    cd ~/catkin_ws
    catkin run_tests
2. To View:
    cd ~/catkin_ws
    catkin_test_results build/icarus_rover_v2

Test Results stored at:  ~/catkin_ws/build/icarus_rover_v2/test_results/icarus_rover_v2/gtest-test_gpio_node.xml




