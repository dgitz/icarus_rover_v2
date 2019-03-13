// Derived class
#include "DiagnosticNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include "Driver/LCDDriver.h"
/*! \class DiagnosticNode DiagnosticNode.h "DiagnosticNode.h"
 *  \brief This is a DiagnosticNode class.  Used for the diagnostic_node node.
 *
 */
class DiagnosticNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "diagnostic_node";

	const uint8_t MAJOR_RELEASE_VERSION = 3;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 1-December-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = DIAGNOSTIC_NODE;
	~DiagnosticNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	DiagnosticNodeProcess* get_process() { return process; }
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
	eros::diagnostic rescan_topics(eros::diagnostic diag);
	void heartbeat_Callback(const eros::heartbeat::ConstPtr& msg,const std::string &topicname);
	void resource_Callback(const eros::resource::ConstPtr& msg,const std::string &topicname);
	void diagnostic_Callback(const eros::diagnostic::ConstPtr& msg,const std::string &topicname);
	void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
	//Support Functions
    /*! \brief  Log RAM and CPU Usage */
	bool log_resources();


	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	DiagnosticNodeProcess *process;
	ofstream ram_used_file;
	ofstream cpu_used_file;
	ofstream ram_free_file;
	ofstream cpu_free_file;
	std::vector<ros::Subscriber> resource_subs;
	std::vector<ros::Subscriber> diagnostic_subs;
	std::vector<ros::Subscriber> heartbeat_subs;
	LCDDriver lcd;
	ros::Subscriber armed_state_sub;
	bool logging_initialized;
	uint8_t last_armedstate;

};
