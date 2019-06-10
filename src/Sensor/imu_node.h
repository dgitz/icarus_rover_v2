// Derived class
#include "IMUNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class IMUNode IMUNode.h "IMUNode.h"
 *  \brief This is a IMUNode class.  Used for the imu_node node.
 *
 */
class IMUNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "imu_node";

	const uint8_t MAJOR_RELEASE_VERSION = 0;
	const uint8_t MINOR_RELEASE_VERSION = 1;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 26-Apr-2019";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = POSE_NODE;
	~IMUNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	IMUNodeProcess* get_process() { return process; }
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
	bool new_devicemsg(std::string query,eros::device t_device,eros::leverarm t_leverarm);
	//Support Functions

	//Printing Functions
	void print_3x3_matricies(std::string devicename,IMUNodeProcess::RotationMatrix mat);


	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	ros::ServiceClient srv_leverarm;
    std::vector<IMUDriver> imu_drivers;
    std::vector<ros::Publisher> imu_pubs;
	std::vector<ros::Publisher> imutemp_pubs;

	IMUNodeProcess *process;

};
