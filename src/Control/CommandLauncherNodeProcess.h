#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
/*! \class CommandLauncherNodeProcess CommandLauncherNodeProcess.h "CommandLauncherNodeProcess.h"
 *  \brief This is a CommandLauncherNodeProcess class.  Used for the commandlauncher_node node.
 *
 */
class CommandLauncherNodeProcess: public BaseNodeProcess {
public:
	//Constants
	//Enums
	//Structs
	struct ProcessCommand
	{
		std::string name;
		bool initialized;
		bool running;
		std::string param_string_1;
		uint32_t param_uint32_1;
		std::string command_text;
		std::string process_name;
		std::string kill_name;
		uint32_t pid;
		int32_t restart_counter;
		int32_t max_restarts;
	};
	struct IPMap
	{
		std::string hostname;
		std::string IPAddress;
	};
	struct PortMap
	{
		std::string name;
		uint32_t port;
	};

	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	std::vector<IPMap> get_ipmap() { return ipmap; }
	std::vector<PortMap> get_portmap() { return portmap; }
	std::vector<ProcessCommand> get_processlist() { return processlist; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	//Support Functions
	std::string lookup_deviceIP(std::string hostname); //Returns "" on failure
	uint32_t lookup_port(std::string portname); //Returns 0 on failure
	bool set_processrunning(std::string name,bool running);
	bool set_processpid(std::string name,uint32_t pid);
	bool set_process_restarted(std::string name);
	std::string get_processinfo();
	//Printing Functions


protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	void init_processlist();
	bool load_configfiles();
	std::vector<ProcessCommand> processlist;
	std::vector<IPMap> ipmap;
	std::vector<PortMap> portmap;
};
