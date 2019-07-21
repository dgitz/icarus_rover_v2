Author: David Gitz
Task: Diagnostics
Nodes:
A. diagnostic_node
TODO:
 - LCD Backlight colors
Usage: This node should be run on 1 device on the Robot.
Purpose: 
1. Report and respond to various health issues with different nodes.
2. Generates ready_to_arm signal

Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_diagnostic_node_process
2.  Hat Unit Tests:
  Compile: 
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Diagnostics/unit_tests/
  >>g++ -lm ../Driver/LCDDriver.cpp test_lcd.cpp -o test_lcd
  
Development History:
17-September-2018 David Gitz
Added support for LCD Module #617003
  
B. safety_node
Usage: This node should be run on 1 device on the Robot, and only a ControlModule.
Purpose:
1. Reads Arm Switch 
2. Publishes Ready To Arm

1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_safety_node_process

C. snapshot_node
TODO:
- Transfer snapshots to requestor
Usage: An instance of this node should be run on every device.  One instance should be a MASTER and the rest should be a SLAVE.  Preferably the device that is running the datalogger_node would be the same one running this node as a MASTER.
Purpose:
1. Create device snapshots for each device

1. Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_snapshot_node_process

  Development History:
  20-July-2019 David Gitz
  Created Node.
