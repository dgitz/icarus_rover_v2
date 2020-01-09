// Derived class
#include "SnapshotNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
//C++ System Fileset
//ROS Base Functionality
//ROS Messages
//Project
/*! \class SnapshotNode SnapshotNode.h "SnapshotNode.h"
 *  \brief This is a SnapshotNode class.  Used for the snapshot_node node.
 *
 */
class SnapshotNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "snapshot_node";

	const uint8_t MAJOR_RELEASE_VERSION = 4;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 11-May-2019";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = CONTROLLER_NODE;
	~SnapshotNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	SnapshotNodeProcess* get_process() { return process; }
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
	void truthpose_Callback(const eros::pose::ConstPtr& msg);
	void PPS1_Callback(const std_msgs::Bool::ConstPtr& t_msg);
	void Command_Callback(const eros::command::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,eros::device t_device);
	bool snapshot_service(eros::srv_snapshotstate::Request &req,
					eros::srv_snapshotstate::Response &res);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceServer snapshot_srv;
	ros::ServiceClient srv_device;
	ros::Publisher snapshotstate_pub;
	ros::Subscriber truthpose_sub;
	SnapshotNodeProcess *process;
	SnapshotNodeProcess::SnapshotState prev_snapshot_state;
	SnapshotNodeProcess::SnapshotState snapshot_state;
	ros::ServiceClient srv_snapshotstate;
	ros::Publisher datalogger_snapshot_pub;

};
