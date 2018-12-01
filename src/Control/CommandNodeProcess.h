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
		icarus_rover_v2::command command;
		double rate_hz;
		double lasttime_ran;
		bool send_me;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	icarus_rover_v2::diagnostic finish_initialization();
	icarus_rover_v2::diagnostic init_readytoarm_list(std::vector<std::string> topics);
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	icarus_rover_v2::command get_currentcommand()
	{
		icarus_rover_v2::command c = current_command;
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
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
	icarus_rover_v2::diagnostic new_user_commandmsg(const icarus_rover_v2::command::ConstPtr& msg);
	icarus_rover_v2::diagnostic new_targetmsg(std::string target);
	void new_readytoarmmsg(std::string topic, bool value);
	//Support Functions
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	std::string map_RoverCommand_ToString(int v);
	std::vector<CommandNodeProcess::ReadyToArm> get_ReadyToArmList() { return ReadyToArmList; }
	std::vector<icarus_rover_v2::command> get_PeriodicCommands();
	icarus_rover_v2::diagnostic get_disarmedreason();
	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();

	icarus_rover_v2::diagnostic init_PeriodicCommands();

	long ms_timer;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
	double time_diff(struct timeval timea,struct timeval timeb);
	std::vector<ReadyToArm> ReadyToArmList;
	int armeddisarmed_state;
	int node_state;
	icarus_rover_v2::command current_command;
	icarus_rover_v2::command last_command;
	std::vector<icarus_rover_v2::command> command_history;
	double batterylevel_perc;
	std::vector<PeriodicCommand> periodic_commands;
	std::string disarmed_reason;
};
