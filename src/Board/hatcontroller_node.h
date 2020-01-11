// Derived class
#include "HatControllerNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/ioctl.h>
#include <dirent.h>
#include <sys/types.h>
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <wiringPiI2C.h>
#include "Driver/ServoHatDriver.h"
#include "Driver/TerminalHatDriver.h"
#include "Driver/GPIOHatDriver.h"
#include <i2cmessage.h>
/*! \class HatControllerNode HatControllerNode.h "HatControllerNode.h"
 *  \brief This is a HatControllerNode class.  Used for the hatcontroller_node node.
 *
 */
class HatControllerNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "sample_node";

	const uint8_t MAJOR_RELEASE_VERSION = 2;
	const uint8_t MINOR_RELEASE_VERSION = 2;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 11-January-2020";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = GPIO_NODE;
	~HatControllerNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	HatControllerNodeProcess* get_process() { return process; }
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
	void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
	void DigitalOutput_Callback(const eros::pin::ConstPtr& msg);
	void PwmOutput_Callback(const eros::pin::ConstPtr& msg);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	HatControllerNodeProcess *process;

	std::vector<ServoHatDriver> ServoHats;
	std::vector<GPIOHatDriver> GPIOHats;
	TerminalHatDriver TerminalHat;
	I2CMessageHandler *i2cmessagehandler;
	std::vector<ros::Publisher> signal_sensor_pubs;
	std::vector<ros::Publisher> digitalinput_pubs;
	std::vector<std::string> signal_digitalinput_names;
	std::vector<ros::Subscriber> pwmoutput_subs;
	std::vector<ros::Subscriber> digitaloutput_subs;
	ros::Time last_pwmoutput_sub_time;
	std::vector<ros::Subscriber> diagnostic_subs;
	ros::Time last_diagnostic_sub_time;
	ros::Time last_digitaloutput_time;
	ros::Subscriber armed_state_sub;
	ros::Publisher ready_to_arm_pub;

};
