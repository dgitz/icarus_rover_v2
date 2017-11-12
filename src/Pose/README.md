Author: David Gitz
Task: Pose
Nodes:
A. pose_node
Usage: This node should be run on 1 device, preferably on the same local LAN as the other position sensors.
Configuration:
 * "Attitude_topic": Topic that roscopter/Attitude is being published on.  Should be more generic in the future.
 * "KF_Yaw_sigma_meas": The sigma measurable parameter used in the Yaw Kalman Filter.
 * "KF_Yaw_sigma_model": The sigma model parameter used in the Yaw Kalman Filter.
 * "KF_Yawrate_sigma_meas": The sigma measurable parameter used in the Yawrate Kalman Filter.
 * "KF_Yawrate_sigma_model": The sigma model parameter used in the Yawrate Kalman Filter.

Purpose: Computes high rate vehicle pose from different sensors.
Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_pose_node_process

Loops:
 * Loop1: Process Update
 * Loop2: Pose Publish

Reference:
Simulated using Octave-GUI: C:\SharedDrive\Dropbox\ICARUS\RoverV2\SIMULATION\Pose
Documentation History:
29-June-2017
Added all code, documentation.  

B. imu_node
Usage: This node should be run on any device that has an IMU installed.  Only 1 instance of this node per device is supported.
Configuration:

Purpose: Outputs high rate imu data from different sensors

Unit Tests:
1. Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_imu_node_process
  
Loops:

Documentation History:
12-Nov-2017
Created node.