Author: David Gitz
Task: Control
Nodes:
A. master_node
Usage:
This node should be run on every device.

Purpose:
1. Reads DeviceFile.xml and publishes icarus_rover_v2::device info for all nodes on current Device.
2. Monitors Device Temperature, if supported.
3. Publishes device's available resource information
4. Publishes device's loadfactor and uptime

B. command_node
Usage: This device should be run on 1 device total.

Purpose:
1. Generates commands and publishes to icarus_rover_v2::command topic
2. Reads */ready_to_arm topics
3. Reads /user_armcommand
4. Reads script files and executes commands

C: Command Launcher
Usage: This node should be run on any device that requires external commands to be launched.
Currently supported Commands:
 * CameraStream: Runs gst-launch to send video from /dev/video0 over UDP
Configuration:
 * Loop1: Checks Commands and relaunches as necessary.
 * Loop2: 
 * CameraStream: <DeviceIP>:<Port> The Device IPAddress and Port that should be used to send the Camera Stream to.  NOTE: The DeviceIP must be found in the DeviceFile.xml file, and the Port must be
   in the MiscConfig.xml file.  Currently raw IP address's and ports are not supported.
   
Purpose:
1. Launches external commands outside of ROS.

Unit Tests:
1. To Build/Run:
    cd ~/catkin_ws
    catkin_make catkin_make run_tests_icarus_rover_v2_gtest_test_commandlauncher_node_process
2. To View:
    cd ~/catkin_ws
    catkin_test_results build/icarus_rover_v2

D: Navigation Node
Usage: This node should be run on one device
Purpose: Perform navigation tasks and publish the /state topic


Test Results stored at:  ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_commandlauncher_node_process.xml

Documentation History:

Documentation History:
5-July-2017
Created Process, Node and Unit Test.  Tested on Rover with an AutoSteer application.  Found that yaw rate (from Pose Node), Joystick (probably not) or Steer Servo fairly unresponsive and will result in either minimal corrections or oscillations.  Going to pause for now, until I have something better to test on.
8-October-2017
Finalized node, process and unit test for initial command.


E: Control Group Node
Usage: 
This Node should be run on 1 device only.
Purpose: 
- Executes Control Loops on configured Input Signals and Output Pins based on configurable Gains
Stats:
- Loop Time with 1 ControlGroup and setting 2 Inputs: 7.3705 nS (x86_64)  12.7 nS (armv7l)
ToDo:
- PID Output Updates and unit test results (DONE)
- Initialization: What happens when any input signal is not received (DONE)
- Output Pin (DONE)
- Add query/response message for Pin Definitions to Master Node (DONE)
- Hook Class to Node (DONE)
- Determine timing rates on ControlModules (DONE)
- Hook up DriverStation GUI Calibration Mode to Node (DONE)
Documentation History:
30-Sep-2019 David Gitz
Work Complete on this Node.