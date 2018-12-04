Author: David Gitz
Task: Sandbox-Communication
Support Programs
A. Test SerialComm
Purpose: Test Serial Port Communication

  >>g++ -lm ../../../util/serialmessage.cpp test_serialcomm.cpp -o test_serialcomm

Documentation History:
19-Nov-2017 David Gitz
Created program

B. test_spicomm
Purpose: Test SPI Communication
  Compile:
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Communication/unit_tests/
  >>g++ -o test_spicomm test_spicomm.cpp spicomm.cpp  
  
Documentation History:
24-January-2018 David Gitz
Created program

Nodes:
A. speaker_node
Usage:
This node should be run on 1 device, which has a audio speaker output.
Purpose:
Speaks in Human Voice from text in the following topics:
 * icarus_rover_v2::diagnostic Anything Higher than the INFO Level will speak the Level, the Node Name and the Message Description.