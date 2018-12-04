Author: David Gitz
Task: User_Control
Nodes:
A. audio_node
Requirements:
	sudo apt-get install mediainfo mpg321
Usage: This node should be run on whatever device has an audio input (i.e. a microphone) and analog audio output attached.

To generate new audio files go to: http://www.fromtexttospeech.com/ and use voice "Daisy"
Purpose:
1. Reads audio input periodically and stores wav files for analysis
2. Plays audio files based on input triggers

Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_icarus_rover_v2_gtest_test_audio_node_process

Test Results stored at:  ~/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_audio_node_process.xml
