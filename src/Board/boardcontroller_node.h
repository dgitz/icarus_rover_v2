// Derived class
#include "BoardControllerNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class SampleNode SampleNode.h "SampleNode.h"
 *  \brief This is a SampleNode class.  Used for the sample_node node.
 *
 */
class BoardControllerNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "boardcontroller_node";

	const uint8_t MAJOR_RELEASE_VERSION = 1;
	const uint8_t MINOR_RELEASE_VERSION = 2;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 11-January-2020";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = GPIO_NODE;

	const uint8_t BOARD_ID = 0; //Only 1 SPI Device Currently supported
	~BoardControllerNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	BoardControllerNodeProcess* get_process() { return process; }
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters
	 *
	 */
	eros::diagnostic read_launchparameters();
	/*! \brief Setup other pubs/subs, other node specific init stuff
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	bool run_001hz();
	bool run_01hz();
	bool run_01hz_noisy();
	bool run_1hz();
	bool run_10hz();
	bool run_loop1();
	bool run_loop2();
	bool run_loop3();
	//Attribute Functions
	//Message Functions
	void PPS1_Callback(const std_msgs::Bool::ConstPtr& t_msg);
	void Command_Callback(const eros::command::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,eros::device t_device);
	int spiTxRx(unsigned char txDat);
	int sendMessageQuery(unsigned char query, unsigned char * inputbuffer);
	int sendMessageCommand(unsigned char command,unsigned char * outputbuffer);
	void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	BoardControllerNodeProcess *process;
	int spi_device;
	SPIMessageHandler *spimessagehandler;
	std::vector<ros::Publisher> signal_sensor_pubs;
	uint32_t passed_checksum;
	uint32_t failed_checksum;
	ros::Subscriber armed_state_sub;

};
