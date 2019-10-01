Author: David Gitz
Task: Board
Support Programs
A. Test ServoHat
B. Test TerminalHatDriver

Nodes:
A. hatcontroller_node
Usage: This node should be run on every ControlModule (Raspberry Pi).
Configuration:
 * "analyze_timing": Diagnostic will report how long a pin took to set, based on the origination timestamp.  Default=false.
Supported PN's:
  * GPIOHat: 100007
  * ServoHat: 625004
  * TerminalHat: 625005
Purpose: Controls all directly connected Hats/Capes.
Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_hatcontroller_node_process

Loops:
 * Loop1: Process Update, ServoHat(s) Update
 * Loop2: Ready To Arm Publish
 
Documentation History:
3-June-2017
Added analyze_timing functionality.
    
B. boardcontroller_node
Usage: This node should be run on every ControlModule with a connected Arduino Board.
Supported PN's:
  * 100005
Configuration:
Currently only 1 SPI Device is supported, at dev/spidev0.0
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
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_boardcontroller_node_process
2. Arduino Board Unit Tests:
  Compile:
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Board/unit_tests/
  >>g++ -lm -lwiringPi ../../../util/spimessage.cpp test_arduinoboard.cpp -o test_arduinoboard

Documentation History:
24-July-2018
Added functionality to control LED Strip through ArduinoBoard.
