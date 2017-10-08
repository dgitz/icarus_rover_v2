Author: David Gitz
Task: Control
Nodes:
A. master_node
Usage:
This node should be run on every device.

Purpose:
1. Reads DeviceFile.xml and publishes icarus_rover_v2::device info for all nodes on current Device.
2. Monitors Device Temperature, if supported.
3. Publishes device's available resource information.

B. command_node
Usage: This device should be run on 1 device total.

Purpose:
1. Generates commands and publishes to icarus_rover_v2::command topic
2. Reads */ready_to_arm topics
3. Reads /user_armcommand

C. Arm Controller
Usage:
This node should be run for every Arm on the Robot.

Purpose:
1. Read in the desired pose of the gripper of each Arm, compute a path for the Arm and generate the target joint angles.

D. io_node
Usage: This node should be run for each ControlModule.
Purpose: 
1. Controls the GPIO pins that are hooked directly to the ControlModule.
2. Either drives Arm Pin or reads Arm Pin depending on Configuration.

E. mavlink_node
Purpose: Connects to APM to drive PWM Outputs and Read Sensor Data

F: Auto Drive
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

G: Command Launcher
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

Test Results stored at:  ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_commandlauncher_node_process.xml

Documentation History:
5-July-2017
Created Process, Node and Unit Test.  Tested on Rover with an AutoSteer application.  Found that yaw rate (from Pose Node), Joystick (probably not) or Steer Servo fairly unresponsive and will result in either minimal corrections or oscillations.  Going to pause for now, until I have something better to test on.

G: Command Launcher
Usage: This node should be run on any device that requires external commands to be launched.
Currently supported Commands:
 * CameraStream: Runs gst-launch to send video from /dev/video0 over UDP
Configuration:
 * Loop1: Checks Commands and relaunches as necessary.
 * Loop2: 
 * CameraStreamPort: <Port> The Port that should be used to send the Camera Stream to.  NOTE: The specified PortName must be
   in the MiscConfig.xml file.

Current Commands that are launched:
 * raspivid/gstreamer: gstreamer pipeline is launched that creates a camera TCP stream.  
   
Purpose:
1. Launches external commands outside of ROS.

Unit Tests:
1. To Build/Run:
    cd ~/catkin_ws
    catkin_make catkin_make run_tests_icarus_rover_v2_gtest_test_commandlauncher_node_process
2. To View:
    cd ~/catkin_ws
    catkin_test_results build/icarus_rover_v2

Test Results stored at:  ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_commandlauncher_node_process.xml

Documentation History:
8-October-2017
Finalized node, process and unit test for initial command.


