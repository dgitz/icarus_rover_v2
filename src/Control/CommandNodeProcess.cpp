#include "CommandNodeProcess.h"
eros::diagnostic  CommandNodeProcess::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	diag = init_PeriodicCommands();
	ms_timer = 0;
	timeout_value_ms = 0;
	armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	ReadyToArmList.clear();
	ready_to_arm = false;
	batterylevel_perc = 0.0;
	current_command.Command = ROVERCOMMAND_BOOT;
	last_command = current_command;
	node_state = NODESTATE_BOOTING;
	disarmed_reason = "No Information";
	diagnostic = diag;
	return diagnostic;
}
eros::diagnostic CommandNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		ready = true;

	}
	eros::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	if(ready == true)
		{
			node_state = NODESTATE_RUNNING;
			current_command.Command = ROVERCOMMAND_NONE;
		}


		node_state = NODESTATE_RUNNING; //Hack
		batterylevel_perc = 0.0;
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

		ready_to_arm = temp;
		if(ready_to_arm == false)
		{
			armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
		}
		else if(ready_to_arm == true)
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

	if(diag.Level <= NOTICE)
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running.";

	}
	diagnostic = diag;
	return diag;
}
eros::diagnostic CommandNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = diagnostic;
	return diag;
}
std::vector<eros::diagnostic> CommandNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	return diaglist;
}
std::vector<eros::diagnostic> CommandNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
	bool status = true;

	if (status == true) {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED.";
		diaglist.push_back(diag);
	} else {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED.";
		diaglist.push_back(diag);
	}
	return diaglist;
}
eros::diagnostic CommandNodeProcess::init_readytoarm_list(std::vector<std::string> topics)
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
eros::diagnostic CommandNodeProcess::new_user_commandmsg(const eros::command::ConstPtr& msg)
{
	if((msg->Command == ROVERCOMMAND_ARM) || (msg->Command == ROVERCOMMAND_DISARM))
	{
		if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
		{
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = ERROR;
			diagnostic.Description = "Armed Status is DISARMED AND CANNOT ARM!";
			diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
			return diagnostic;
		}
		last_command = current_command;
		current_command = convert_fromptr(msg);
		if(msg->Command == ROVERCOMMAND_ARM)
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
		sprintf(tempstr,"User Command: %d Not supported.",msg->Command);
		diagnostic.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
		return diagnostic;
	}
}

eros::diagnostic CommandNodeProcess::get_disarmedreason()
{
	eros::diagnostic diag = diagnostic;
	if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
	{
		diag.Diagnostic_Type = REMOTE_CONTROL;
		diag.Diagnostic_Message = ROVER_DISARMED;
		diag.Level = WARN;
		diag.Description = disarmed_reason;
	}
	return diag;
}
eros::diagnostic CommandNodeProcess::new_targetmsg(std::string target)
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

std::vector<eros::command> CommandNodeProcess::get_PeriodicCommands()
{
	std::vector<eros::command> sendlist;
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
eros::diagnostic CommandNodeProcess::init_PeriodicCommands()
{
	std::vector<PeriodicCommand> commands;
	{
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		cmd.Option1 = LEVEL1;
		cmd.Description = "Low-Level Diagnostics";
		PeriodicCommand period_cmd;
		period_cmd.command = cmd;
		period_cmd.rate_hz = 10.0;
		commands.push_back(period_cmd);
	}

	{
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		cmd.Option1 = LEVEL2;
		cmd.Description = "Medium-Level Diagnostics";
		PeriodicCommand period_cmd;
		period_cmd.command = cmd;
		period_cmd.rate_hz = 1.0;
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
