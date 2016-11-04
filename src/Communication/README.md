Author: David Gitz
Task: Communication
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

C. topicremapper_node
Usage: This node should be run on one device on the Robot.
Purpose:
1. Reads how it should remap based off TopicMap.xml file
2. Maps the following topics to other topics:
 * sensor_msgs::Joy.axis -> icarus_rover_v2::pin.Value Scaled: [0,255]
 * sensor_msgs::Joy.button -> icarus_rover_v2::pin.Value

Utilities:
A. Message Generation
Holdover from eROS Integration.  See https://github.com/dgitz/eROS/wiki/eROS for more info

