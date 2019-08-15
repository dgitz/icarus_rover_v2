Author: David Gitz
Task: Communication
Support Programs
Documentation History:
24-January-2018 David Gitz
Created program

Nodes:
A. network_transceiver_node
Usage: This node should run on one device on the Robot, with an attached WiFi/LAN Connection
Purpose:
1. Converts ROS Topics to UDP Messages and Transmits to the defined Multicast Address and Port.  For a list of supported messages visit: 
https://github.com/dgitz/icarus_rover_v2/wiki/ICARUS_ROVER_V2_COMM#udp-messages
2. Receives UDP Messages and converts to ROS messages and then Publishes.  For a list of supported messages visit:
https://github.com/dgitz/icarus_rover_v2/wiki/ICARUS_ROVER_V2_COMM#udp-messages
3. Generates ready_to_arm signal

Documentation History
10-Sep-2018 David Gitz
Not able to fix BUG: https://github.com/dgitz/icarus_rover_v2/issues/109 except by moving node to ControlModule2.

B. topicremapper_node
Usage: This node should be run on one device on the Robot.
Purpose:
1. Reads how it should remap based off TopicMap.xml file
2. Maps the following topics to other topics:
 * sensor_msgs::Joy.axis -> icarus_rover_v2::pin,std_msgs/Float32,sensor_msgs/JointState 
 * sensor_msgs::Joy.button -> icarus_rover_v2::pin.Value
3. Uses an OptionMode (optional) to map input topics to multiple output topics

C. database_node
ToDo:
Usage: This node should be run on one device on the Robot.
Purpose:  Connects to a SQLite Database and provides a SQL Query Service.
References:
https://www.tutorialspoint.com/sqlite/sqlite_installation.htm

Instructions:
* Create Database
1. Export Access Database to CSV's:  Open Database and click "Export" on Switchboard.
2. On Linux, run the command:
  >>./csv_to_sqldb.py -c ~/Dropbox/FASTRobotics/FASTDatabase/ ~/config/configdatabase.db

* Manually run a SQL command
1. Run the database_node and master_node
2. Execute the following command:
  >>rosservice call /ConfigDatabase/srv_sql '{type: 3,cmd: SELECT * FROM FASTDatabaseTable_Item}'

Documentation History:
12-Aug-2019 David Gitz
Created Node.
