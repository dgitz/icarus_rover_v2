#include "command_node_process.h"
/*! \brief Constructor
 */
CommandNodeProcess::CommandNodeProcess(std::string _base_node_name,std::string _node_name)
{
	base_node_name = _base_node_name;
	node_name = _node_name;
	unittest_running = false;
	run_time = 0.0;
	initialized = false;
	ready = false;

	ms_timer = 0;
	timeout_value_ms = 0;
	gettimeofday(&init_time,NULL);
	armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	ReadyToArmList.clear();
	readytoarm = false;
	batterylevel_perc = 0.0;
	current_command.Command = ROVERCOMMAND_BOOT;
	last_command = current_command;
	node_state = NODESTATE_BOOTING;
	disarmed_reason = "No Information";
	//armedcommand = ARMEDCOMMAND_DISARM;
}
/*! \brief Deconstructor
 */
CommandNodeProcess::~CommandNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic CommandNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	diagnostic = init_PeriodicCommands();
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic CommandNodeProcess::update(double dt)
{
	run_time += dt;
	if((mydevice.BoardCount == 0) and (mydevice.SensorCount == 0))
	{
		if(initialized == true) { ready = true; }
	}
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(ready == true)
	{
		node_state = NODESTATE_RUNNING;
		current_command.Command = ROVERCOMMAND_NONE;
	}


	node_state = NODESTATE_RUNNING; //Hack
	batterylevel_perc = 0.0;
	/*
	if((node_state == NODESTATE_RUNNING) && (current_command.Command == ROVERCOMMAND_NONE))
	{
		if(batterylevel_perc < BATTERYLEVEL_TO_RECHARGE)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY;
			current_command.Option1 = 0;
			current_command.Option2 = 0;
			current_command.Option3 = 0;
			node_state = NODESTATE_SEARCHING_FOR_RECHARGE_FACILITY;
		}
		else
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_NONE;
			current_command.Option1 = 0;
			current_command.Option2 = 0;
			current_command.Option3 = 0;
		}
	}
	else if(node_state == NODESTATE_ACQUIRING_TARGET)
	{
		last_command = current_command;
		current_command.Command = ROVERCOMMAND_ACQUIRE_TARGET;
		current_command.Option1 = 0;
		current_command.Option2 = 0;
		current_command.Option3 = 0;
	}
	if(batterylevel_perc > BATTERYLEVEL_RECHARGED)
	{
		node_state = NODESTATE_RUNNING;
	}
	 */
	bool temp = true;
	char tempstr[512*(int)ReadyToArmList.size()];
	memset(tempstr,0,strlen(tempstr));
	if(ReadyToArmList.size() == 0)
	{

		disarmed_reason = "Ready To Arm List Size == 0";
		temp = false;
	}
	else
	{
		for(int i = 0; i < ReadyToArmList.size();i++)
		{
			if(ReadyToArmList.at(i).ready_to_arm == false)
			{
				temp = false;
				sprintf(tempstr,"%sTopic: %s Reports is Unable to Arm.\n",tempstr,ReadyToArmList.at(i).topic.c_str());
			}
		}
		if(temp == false)
		{
			disarmed_reason = std::string(tempstr);
		}
	}

	readytoarm = temp;
	if(readytoarm == false)
	{
		armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	}
	else if(readytoarm == true)
	{

		if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
		{
			disarmed_reason = "None";
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
		}
	}
	for(std::size_t i = 0; i < periodic_commands.size(); i++)
	{
		double time_to_run = periodic_commands.at(i).lasttime_ran + (1.0/periodic_commands.at(i).rate_hz);
		if(time_to_run <= run_time)
		{
			periodic_commands.at(i).send_me = true;
			periodic_commands.at(i).lasttime_ran = run_time;
		}
	}


	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Node Running";
	diagnostic = diag;
	return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic CommandNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(mydevice.DeviceName == myhostname)
	{

	}
	return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> CommandNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command ==  ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if(cmd.Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if(cmd.Option1 == LEVEL2)
		{
			diaglist = check_program_variables();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL4)
		{
		}
	}
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> CommandNodeProcess::check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag=diagnostic;
	bool status = true;

	if(status == true)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}
icarus_rover_v2::diagnostic CommandNodeProcess::init_readytoarm_list(std::vector<std::string> topics)
{
	for(int i = 0; i < topics.size();i++)
	{
		ReadyToArm newarm;
		newarm.topic = topics.at(i);
		newarm.ready_to_arm = false;
		newarm.time_since_lastrx = 0.0;
		ReadyToArmList.push_back(newarm);
	}
	diagnostic.Diagnostic_Type = SOFTWARE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Initialized Ready To Arm List.";
	return diagnostic;
}
void CommandNodeProcess::new_readytoarmmsg(std::string topic, bool value)
{
	for(int i = 0; i < ReadyToArmList.size();i++)
	{
		if(ReadyToArmList.at(i).topic == topic)
		{

			ReadyToArmList.at(i).ready_to_arm = value;
			ReadyToArmList.at(i).time_since_lastrx = run_time;
		}
	}
}
icarus_rover_v2::diagnostic CommandNodeProcess::new_user_commandmsg(icarus_rover_v2::command msg)
{
	if((msg.Command == ROVERCOMMAND_ARM) || (msg.Command == ROVERCOMMAND_DISARM))
	{
		if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
		{
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = FATAL;
			diagnostic.Description = "Armed Status is DISARMED AND CANNOT ARM!";
			diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
			return diagnostic;
		}
		last_command = current_command;
		current_command = msg;
		if(msg.Command == ROVERCOMMAND_ARM)
		{
			armeddisarmed_state = ARMEDSTATUS_ARMED;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = NOTICE;
			diagnostic.Description = "Rover is ARMED";
			diagnostic.Diagnostic_Message = ROVERCOMMAND_ARM;
			disarmed_reason = "None";
			return diagnostic;
		}
		else//msg.Command == ROVERCOMMAND_DISARM
		{
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = NOTICE;
			diagnostic.Description = "Rover is DISARMED";
			diagnostic.Diagnostic_Message = ROVERCOMMAND_DISARM;
			disarmed_reason = "None";
			return diagnostic;
		}
	}
	else
	{
		diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		diagnostic.Level = WARN;
		char tempstr[512];
		sprintf(tempstr,"User Command: %d Not supported.",msg.Command);
		diagnostic.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
		return diagnostic;
	}
}

icarus_rover_v2::diagnostic CommandNodeProcess::get_disarmedreason()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
	{
		diag.Diagnostic_Type = REMOTE_CONTROL;
		diag.Diagnostic_Message = ROVER_DISARMED;
		diag.Level = WARN;
		diag.Description = disarmed_reason;
	}
	return diag;
}
icarus_rover_v2::diagnostic CommandNodeProcess::new_targetmsg(std::string target)
{
	if(target == "outlet")
	{
		if(node_state == NODESTATE_SEARCHING_FOR_RECHARGE_FACILITY)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY;
			node_state = NODESTATE_ACQUIRING_TARGET;
		}
		diagnostic.Diagnostic_Type = TARGET_ACQUISITION;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Level = INFO;
		char tempstr[512];
		sprintf(tempstr,"Found target: %s",target.c_str());
		diagnostic.Description = std::string(tempstr);
		return diagnostic;

	}
	else if(target == "unknown")
	{
		diagnostic.Diagnostic_Type = TARGET_ACQUISITION;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Description = "No Target Found";
		return diagnostic;
	}
	else
	{
		diagnostic.Diagnostic_Type = TARGET_ACQUISITION;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Level = WARN;
		char tempstr[512];
		sprintf(tempstr,"Found target: %s But not currently supported",target.c_str());
		diagnostic.Description = std::string(tempstr);
		return diagnostic;
	}
}

std::vector<icarus_rover_v2::command> CommandNodeProcess::get_PeriodicCommands()
{
	std::vector<icarus_rover_v2::command> sendlist;
	for(std::size_t i = 0; i < periodic_commands.size(); i++)
	{
		if(periodic_commands.at(i).send_me == true)
		{
			sendlist.push_back(periodic_commands.at(i).command);
			periodic_commands.at(i).send_me = false;
		}
	}
	return sendlist;
}
icarus_rover_v2::diagnostic CommandNodeProcess::init_PeriodicCommands()
{
	std::vector<PeriodicCommand> commands;
	{
		icarus_rover_v2::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		cmd.Option1 = LEVEL2;
		cmd.Description = "Mid-Level Diagnostics";
		PeriodicCommand period_cmd;
		period_cmd.command = cmd;
		period_cmd.rate_hz = 1.0;
		commands.push_back(period_cmd);
	}

	{
		icarus_rover_v2::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		cmd.Option1 = LEVEL3;
		cmd.Description = "High-Level Diagnostics";
		PeriodicCommand period_cmd;
		period_cmd.command = cmd;
		period_cmd.rate_hz = 0.001;
		commands.push_back(period_cmd);
	}



	for(int i = 0; i < commands.size(); i++)
	{
		commands.at(i).lasttime_ran = 0.0;
		commands.at(i).send_me = true;
	}
	periodic_commands = commands;
	diagnostic.Diagnostic_Type = SOFTWARE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Initialized Periodic Command List";
	return diagnostic;
}
double CommandNodeProcess::time_diff(struct timeval timea, struct timeval timeb)
{
	long mtime, seconds, useconds;
	seconds  = timeb.tv_sec  - timea.tv_sec;
	useconds = timeb.tv_usec - timea.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	return (double)(mtime)/1000.0;
}

std::string CommandNodeProcess::map_RoverCommand_ToString(int command)
{

	switch(command)
	{
	case ROVERCOMMAND_UNDEFINED: 						return "Undefined";							break;
	case ROVERCOMMAND_BOOT:								return "Boot";								break;
	case ROVERCOMMAND_NONE:								return "No Command";						break;
	case ROVERCOMMAND_RUNDIAGNOSTIC:					return "Run Diagnostic";					break;
	case ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY:		return "Search For Recharge Facility";		break;
	case ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY:	return "Stop Search for Recharge Facility";	break;

	default:
		std::string tempstr;
		tempstr = "Command: " + boost::lexical_cast<std::string>(command) + " Not Supported";
		return tempstr;
	}
}
/*! \brief Run Unit Test
 */
std::vector<icarus_rover_v2::diagnostic> CommandNodeProcess::run_unittest()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	if(unittest_running == false)
	{
		unittest_running = true;
		icarus_rover_v2::diagnostic diag=diagnostic;
		bool status = true;
		std::string data;
		std::string cmd = "cd ~/catkin_ws && "
				"bash devel/setup.bash && catkin_make run_tests_icarus_rover_v2_gtest_test_" + base_node_name + "_process >/dev/null 2>&1 && "
				"mv /home/robot/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_" + base_node_name + "_process.xml "
				"/home/robot/catkin_ws/build/test_results/icarus_rover_v2/" + base_node_name + "/ >/dev/null 2>&1";
		system(cmd.c_str());
		cmd = "cd ~/catkin_ws && bash devel/setup.bash && catkin_test_results build/test_results/icarus_rover_v2/" + base_node_name + "/";
		FILE * stream;

		const int max_buffer = 256;
		char buffer[max_buffer];
		cmd.append(" 2>&1");
		stream = popen(cmd.c_str(), "r");
		if (stream)
		{
			if (!feof(stream))
			{
				if (fgets(buffer, max_buffer, stream) != NULL) { data.append(buffer); }
				pclose(stream);
			}
		}
		std::vector<std::string> strs;
		std::size_t start = data.find(":");
		data.erase(0,start+1);
		boost::split(strs,data,boost::is_any_of(",: "),boost::token_compress_on);
		if(strs.size() < 6)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[1024];
			sprintf(tempstr,"Unable to process Unit Test Result: %s",data.c_str());
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			return diaglist;
		}
		int test_count = std::atoi(strs.at(1).c_str());
		int error_count = std::atoi(strs.at(3).c_str());
		int failure_count = std::atoi(strs.at(5).c_str());
		if(test_count == 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Test Count: 0.";
			diaglist.push_back(diag);
			status = false;
		}
		if(error_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr,"Error Count: %d",error_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if(failure_count > 0)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			char tempstr[512];
			sprintf(tempstr,"Failure Count: %d",failure_count);
			diag.Description = std::string(tempstr);
			diaglist.push_back(diag);
			status = false;
		}
		if(status == true)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = NOTICE;
			diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
			diag.Description = "Unit Test -> PASSED";
			diaglist.push_back(diag);
		}
		else
		{
			diag.Diagnostic_Type = SOFTWARE;
			uint8_t highest_error = INFO;
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				if(diaglist.at(i).Level > highest_error)
				{
					highest_error = diaglist.at(i).Level;
				}
			}
			diag.Level = highest_error;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Unit Test -> FAILED";
			diaglist.push_back(diag);
		}

		unittest_running = false;
	}
	else
	{

		icarus_rover_v2::diagnostic diag=diagnostic;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = "Unit Test -> IS STILL IN PROGRESS";
		diaglist.push_back(diag);
	}
	return diaglist;
}
