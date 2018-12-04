// Derived class
#include "AudioNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class AudioNode AudioNode.h "AudioNode.h"
 *  \brief This is a AudioNode class.  Used for the audio_node node.
 *
 */
class AudioNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "audio_node";

	const uint8_t MAJOR_RELEASE_VERSION = 1;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 4-December-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = COMMUNICATION_NODE;
	~AudioNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	AudioNodeProcess* get_process() { return process; }
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters
	 *
	 */
	icarus_rover_v2::diagnostic read_launchparameters();
	/*! \brief Setup other pubs/subs, other node specific init stuff
	 *
	 */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	bool run_001hz();
	bool run_01hz();
	bool run_1hz();
	bool run_10hz();
	bool run_loop1();
	bool run_loop2();
	bool run_loop3();
	//Attribute Functions
	//Message Functions
	void PPS1_Callback(const std_msgs::Bool::ConstPtr& t_msg);
	void Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,icarus_rover_v2::device t_device);
	void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	AudioNodeProcess *process;
	ros::Subscriber armed_state_sub;

};
