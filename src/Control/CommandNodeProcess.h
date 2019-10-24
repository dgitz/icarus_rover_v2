#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
#include <dirent.h>
//C++ System Files
#include <fstream>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <map>
//ROS Base Functionality
//ROS Messages
#include <rosgraph_msgs/Clock.h>
//Project
/*! \class CommandNodeProcess CommandNodeProcess.h "CommandNodeProcess.h"
 *  \brief This is a CommandNodeProcess class.  Used for the command_node node.
 *
 */
class CommandNodeProcess: public BaseNodeProcess {
public:
	//Constants
	const double BATTERYLEVEL_TO_RECHARGE = 30.0f;
	const double BATTERYLEVEL_RECHARGED = 95.0f;
	const double GAZEBO_PAUSETIME = 2.0f;

	//const double
	//Enums
	enum ScriptCommandMode
	{
		EXECUTION_COUNT=0,
		DURATION=1,
		UNTIL_NEXT=2
	};
	//Structs
	struct ReadyToArm
	{
		std::string Device;
		std::string topic;
		bool ready_to_arm;
		double time_since_lastrx;
	};
	struct PeriodicCommand
	{
		eros::command command;
		double rate_hz;
		double lasttime_ran;
		bool send_me;
	};
	struct ScriptCommand
	{
		eros::command command;
		double command_starttime;
		double command_stoptime;
		ScriptCommandMode execution_mode;
		uint16_t execution_count;
		uint16_t counter;
		double duration;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	eros::diagnostic init_readytoarm_list(std::vector<std::string> topics);
	eros::diagnostic load_loadscriptingfiles(std::string directory); //Use "" for default path, otherwise use specified directory
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	uint8_t get_gazebomessagetopublish() 
	{
		uint8_t v = gazebo_message_topublish;
		gazebo_message_topublish = 0;
		return v;
	}
	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	eros::command get_currentcommand()
	{
		eros::command c = current_command;
		current_command.Command = ROVERCOMMAND_NONE;
		return c;
	}
	std::vector<eros::system_state> get_statelist() { return current_statelist; }
	int get_currentstate() { return node_state; }
	void set_batterylevel_perc(double v) { batterylevel_perc = v; }
	double get_batterylevel_perc() { return batterylevel_perc; }
	int get_armeddisarmed_state() { return armeddisarmed_state; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	eros::diagnostic new_user_commandmsg(const eros::command::ConstPtr& msg);
	eros::diagnostic new_targetmsg(std::string target);
	void new_readytoarmmsg(std::string topic, bool value);
	void new_gazeboclockmsg()
	{
		timesince_lastgazeboclock = run_time;
	}
	void new_gazebo_updaterate(double v)
	{
		gazebo_updaterate = v;
	}
	//Support Functions
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	std::string map_RoverCommand_ToString(uint16_t v);
	uint16_t map_RoverCommand_ToInt(std::string command);
	std::vector<CommandNodeProcess::ReadyToArm> get_ReadyToArmList() { return ReadyToArmList; }
	std::vector<eros::command> get_PeriodicCommands();
	eros::diagnostic get_disarmedreason();
	std::vector<eros::command> get_command_buffer();
	double get_scriptexecutiontime() { return script_execution_time; }
	//Printing Functions
	void print_scriptcommand_list();
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */

	std::vector<eros::diagnostic> check_programvariables();
	eros::diagnostic init_PeriodicCommands();
	void init_StateList();
	std::map<uint16_t,std::string> command_map;
	long ms_timer;
	double gazebo_updaterate;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
	double time_diff(struct timeval timea,struct timeval timeb);
	std::vector<ReadyToArm> ReadyToArmList;
	int armeddisarmed_state;
	int node_state;
	eros::command current_command;
	eros::command last_command;
	std::vector<eros::system_state> current_statelist;
	std::vector<eros::command> command_history;
	double batterylevel_perc;
	std::vector<PeriodicCommand> periodic_commands;
	std::string disarmed_reason;
	double script_execution_time;
	std::vector<ScriptCommand> script_commands;
	std::vector<eros::command> command_buffer;
	uint8_t gazebo_message_topublish;
	double timesince_lastgazeboclock;
};
