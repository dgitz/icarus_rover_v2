// Derived class
#include "DataLoggerNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
#include <boost/regex.hpp>
//ROS Base Functionality
//#include "../../../ros_comm/tools/rosbag/include/rosbag/recorder.h"
//#include "record_ros/record.h"
#include <rosbag/recorder.h>
//ROS Messages
//Project
/*! \class DataLoggerNode DataLoggerNode.h "DataLoggerNode.h"
 *  \brief This is a DataLoggerNode class.  Used for the sample_node node.
 *
 */
class DataLoggerNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "datalogger_node";

	const uint8_t MAJOR_RELEASE_VERSION = 1;
	const uint8_t MINOR_RELEASE_VERSION = 1;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 11-January-2020";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = DIAGNOSTIC_NODE;
	~DataLoggerNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	DataLoggerNodeProcess* get_process() { return process; }
	void thread_loop();
    void run_logger(DataLoggerNode *node);

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
	//Support Functions
    



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	DataLoggerNodeProcess *process;
    
    //rosbag::Recorder recorder;

};
