Author: David Gitz
Task: Sandbox-Control
Nodes:
A. Arm Controller
Usage:
This node should be run for every Arm on the Robot.

Purpose:
1. Read in the desired pose of the gripper of each Arm, compute a path for the Arm and generate the target joint angles.

B. io_node
Usage: This node should be run for each ControlModule.
Purpose: 
1. Controls the GPIO pins that are hooked directly to the ControlModule.
2. Either drives Arm Pin or reads Arm Pin depending on Configuration.

C. mavlink_node
Purpose: Connects to APM to drive PWM Outputs and Read Sensor Data

D: Auto Drive
Usage: This node should be run on 1 Device.
Uses Configuration file: "/home/robot/config/ControlGroup.xml"
Configuration:
 * Loop1: Process Update
 * Loop2: PWM Output Publish
 * Mode: DriverStation,Disabled
   * DriverStation: Reads topic /controlgroup for tuning commands
   * Disabled: No PID Output

Purpose:
1. Performs PID feedback based on Sensor and Command data.

Unit Tests:
1. To Build/Run:
    cd ~/catkin_ws
    catkin_make catkin_make run_tests_icarus_rover_v2_gtest_test_autodrive_node_process
2. To View:
    cd ~/catkin_ws
    catkin_test_results build/icarus_rover_v2

Test Results stored at:  ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_autodrive_node_process.xml