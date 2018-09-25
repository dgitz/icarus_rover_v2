Author: David Gitz
Task: User_Control
Nodes:
A. usercontrol_node - OBSOLETE
Usage: This node should be run on whatever device has a User Input Device.  This node is likely obsolete.
Purpose:
1. Reads a connected joystick and publishes to various topics.

B. teleop_node
Usage:
Purpose:

C. audio_node
Requirements:
	sudo apt-get install mediainfo mpg321
Usage: This node should be run on whatever device has an audio input (i.e. a microphone) and analog audio output attached.
Purpose:
1. Reads audio input periodically and stores wav files for analysis
2. Plays audio files based on input triggers

Test Results stored at:  ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_audio_node_process.xml
