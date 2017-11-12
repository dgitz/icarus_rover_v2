Author: David Gitz
Task: Simulate
Nodes:
A. simulate_node
Usage: This node should be run on 1 device, preferably on the same local LAN as the other position sensors.
Configuration:


Purpose: Simulates vehicle and outputs simulated sensor data for pose computation
Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_simulate_node_process

Loops:
 * Loop1: Process Update
 * Loop2: Sim Pose Publish
 * Loop3: Sim Sensor Publish

Scripts:
Drive: Option1 = Time to perform action.  Option2 = Steer Command (%). Option3 = Throttle Command (%).
Stop: Option1 = Time to perform action.
Reference:

Documentation History:
14-October-2017
Created node.



