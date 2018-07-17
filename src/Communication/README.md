Author: David Gitz
Task: Communication
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

B. network_transceiver_node
Usage: This node should run on one device on the Robot, with an attached WiFi/LAN Connection
Purpose:
1. Converts ROS Topics to UDP Messages and Transmits to the defined Multicast Address and Port.  For a list of supported messages visit: 
https://github.com/dgitz/icarus_rover_v2/wiki/ICARUS_ROVER_V2_COMM#udp-messages
2. Receives UDP Messages and converts to ROS messages and then Publishes.  For a list of supported messages visit:
https://github.com/dgitz/icarus_rover_v2/wiki/ICARUS_ROVER_V2_COMM#udp-messages
3. Generates ready_to_arm signal

C. topicremapper_node
Usage: This node should be run on one device on the Robot.
Purpose:
1. Reads how it should remap based off TopicMap.xml file
2. Maps the following topics to other topics:
 * sensor_msgs::Joy.axis -> icarus_rover_v2::pin,std_msgs/Float32,sensor_msgs/JointState 
 * sensor_msgs::Joy.button -> icarus_rover_v2::pin.Value
3. Uses an OptionMode (optional) to map input topics to multiple output topics

Utilities:
A. Message Generation
Holdover from eROS Integration.  See https://github.com/dgitz/eROS/wiki/eROS for more info

