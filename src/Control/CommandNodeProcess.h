#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
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
	//Enums
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
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	eros::diagnostic init_readytoarm_list(std::vector<std::string> topics);
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	eros::command get_currentcommand()
	{
		eros::command c = current_command;
		current_command.Command = ROVERCOMMAND_NONE;
		return c;
	}
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
	//Support Functions
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	std::string map_RoverCommand_ToString(int v);
	std::vector<CommandNodeProcess::ReadyToArm> get_ReadyToArmList() { return ReadyToArmList; }
	std::vector<eros::command> get_PeriodicCommands();
	eros::diagnostic get_disarmedreason();
	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();

	eros::diagnostic init_PeriodicCommands();

	long ms_timer;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
	double time_diff(struct timeval timea,struct timeval timeb);
	std::vector<ReadyToArm> ReadyToArmList;
	int armeddisarmed_state;
	int node_state;
	eros::command current_command;
	eros::command last_command;
	std::vector<eros::command> command_history;
	double batterylevel_perc;
	std::vector<PeriodicCommand> periodic_commands;
	std::string disarmed_reason;
};
