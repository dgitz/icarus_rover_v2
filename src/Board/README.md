Author: David Gitz
Task: Board
Nodes:
A. hatcontroller_node
Usage: This node should be run on every ControlModule (Raspberry Pi).
Configuration:
 * "analyze_timing": Diagnostic will report how long a pin took to set, based on the origination timestamp.  Default=false.
 
Purpose: Controls all directly connected Hats/Capes.
Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_hatcontroller_node_process
2.  Hat Unit Tests:
  Compile: 
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Board/unit_tests/
  >>g++ -lm -lwiringPi ../Driver/ServoHatDriver.cpp test_servohat.cpp -o test_servohat
  >>g++ -lm -lwiringPi ../Driver/TerminalHatDriver.cpp test_terminalhat.cpp -o test_terminalhat

Loops:
 * Loop1: Process Update, ServoHat(s) Update
 * Loop2: Ready To Arm Publish
 
Documentation History:
3-June-2017
Added analyze_timing functionality.
    
B. boardcontroller_node
Usage: This node should be run on every ControlModule with a connected Arduino Board.
Supported Arduino Boards:
 * Arduino Mega 2560
Configuration:
Installation:
SPI Connections:
		Raspberry Pi 2 Model B	|	Arduino Mega 2560
MOSI			19							51
MISO			21							50	
SCLK			23							52	
CE0/SS			24							53

Purpose: Communicates with an Arduino Board

Unit Tests:
1. Process Unit Tests:

2. SPI Comm Unit Tests:
  Compile:
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Board/unit_tests/
  >>g++ -o test_arduinoboard test_arduinoboard.cpp spicomm.cpp
  
B. gpio_node --> OBSOLETE
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




