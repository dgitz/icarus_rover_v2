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

