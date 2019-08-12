// Derived class
#include "DatabaseNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <sqlite3.h> 
/*! \class DatabaseNode database_node.h "database_node.h"
 *  \brief This is a DatabaseNode class.  Used for the database_node node.
 *
 */
//static int callback(void *data, int argc, char **argv, char **azColName);
using Record = std::vector<std::string>;
using Records = std::vector<Record>;
class DatabaseNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "database_node";

	const uint8_t MAJOR_RELEASE_VERSION = 0;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 12-Aug-2019";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = COMMUNICATION_NODE;
	~DatabaseNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	DatabaseNodeProcess* get_process() { return process; }
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters
	 *
	 */
	eros::diagnostic read_launchparameters();
	eros::diagnostic rescan_topics();
	/*! \brief Setup other pubs/subs, other node specific init stuff
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	static int database_query(void *data, int argc, char **argv, char **azColName);
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


	bool sql_service(eros::srv_sql::Request &req,
					eros::srv_sql::Response &res);
	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	DatabaseNodeProcess *process;
	std::vector<ros::Subscriber> multiple_subs;
	ros::ServiceServer sql_srv;
	sqlite3 *db;
	int db_fd;

};
