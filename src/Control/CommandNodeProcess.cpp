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
	script_execution_time = 0.0;
	command_buffer.clear();
	command_map[ROVERCOMMAND_UNDEFINED] = "UNDEFINED";
	command_map[ROVERCOMMAND_BOOT] = "BOOT";
	command_map[ROVERCOMMAND_NONE] = "NONE";
	command_map[ROVERCOMMAND_RUNDIAGNOSTIC] = "RUN DIAGNOSTIC";
	command_map[ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY] = "SEARCH FOR RECHARGE FACILITY";
	command_map[ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY] = "STOP SEARCH FOR RECHARGE FACILITY";
	command_map[ROVERCOMMAND_ACQUIRE_TARGET] = "ACQUIRE TARGET";
	command_map[ROVERCOMMAND_ARM] = "ARM ROBOT";
	command_map[ROVERCOMMAND_DISARM] = "DISARM ROBOT";
	command_map[ROVERCOMMAND_CONFIGURE] = "CONFIGURE ROBOT";
	command_map[ROVERCOMMAND_RUN] = "RUN";
	command_map[ROVERCOMMAND_STOPMOVEMENT] = "STOP MOVEMENT";
	command_map[ROVERCOMMAND_DRIVECOMMAND] = "DRIVE COMMAND";
	command_map[ROVERCOMMAND_WAIT] = "WAIT";
	diagnostic = diag;
	return diagnostic;
}
eros::diagnostic CommandNodeProcess::update(double t_dt,double t_ros_time)
{
	//NEED MUTEX LOCK
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
std::vector<eros::command> CommandNodeProcess::get_command_buffer()
{
	std::vector<eros::command> command_buffer;
	for(std::size_t i = 0; i < script_commands.size(); ++i)
	{

		if(script_commands.at(i).command_starttime > get_runtime())
		{
			break;
		}
		if((script_commands.at(i).command_stoptime < get_runtime()) and (script_commands.at(i).execution_mode != ScriptCommandMode::EXECUTION_COUNT))
		{
			script_commands.erase(script_commands.begin()+i);
		}
		if(script_commands.size() > 0)
		{
			if(script_commands.at(i).execution_mode == ScriptCommandMode::EXECUTION_COUNT)
			{

				if(script_commands.at(i).counter < script_commands.at(i).execution_count)
				{
					command_buffer.push_back(script_commands.at(i).command);
				}
				else
				{
					script_commands.erase(script_commands.begin()+i);
				}
				script_commands.at(i).counter++;
			}
			else
			{
				command_buffer.push_back(script_commands.at(i).command);
			}
		}

	}
	return command_buffer;
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
uint16_t CommandNodeProcess::map_RoverCommand_ToInt(std::string command)
{
	std::map<std::string,uint16_t> reverse_map;
	std::map<std::string,uint16_t>::iterator it = reverse_map.begin();

	for (auto& x: command_map)
	{
		reverse_map.insert (it, std::pair<std::string,uint16_t>(x.second,x.first));
	}
	it = reverse_map.find(command);
	if (it != reverse_map.end())
	{
		return it->second;
	}
	return ROVERCOMMAND_UNDEFINED;
}
std::string CommandNodeProcess::map_RoverCommand_ToString(uint16_t command)
{
	std::map<uint16_t,std::string>::iterator it;
	it = command_map.find(command);
	if (it != command_map.end())
	{
		return it->second;
	}
	return "UNDEFINED";
}
eros::diagnostic CommandNodeProcess::load_loadscriptingfiles(std::string directory) //Use "" for default path, otherwise use specified directory
{
	eros::diagnostic diag = diagnostic;
	std::vector<std::string> files_to_load;
	DIR *dpdf;
	struct dirent *epdf;

	dpdf = opendir(directory.c_str());
	if (dpdf != NULL)
	{
		while (epdf = readdir(dpdf))
		{
			std::string tempstr = std::string(epdf->d_name);
			if(tempstr.length() <= 4)
			{
				continue;
			}
			files_to_load.push_back(directory + tempstr);
			//printf("Filename: %s",epdf->d_name);
			// std::cout << epdf->d_name << std::endl;
		}
	}
	closedir(dpdf);
	for(std::size_t i = 0; i < files_to_load.size(); ++i)
	{
		std::string file = files_to_load.at(i);
		std::ifstream myfile(file.c_str());
		std::string line;
		int line_counter = 0;
		getline (myfile,line); //Ignore header
		if (myfile.is_open())
		{
			while ( getline (myfile,line) )
			{
				line_counter++;
				if(line.length() == 0)
				{
					continue;
				}
				std::vector<std::string> items;
				boost::split(items,line,boost::is_any_of(","),boost::token_compress_on);
				ScriptCommand cmd;
				uint16_t command_type = map_RoverCommand_ToInt(items.at(0));
				if(command_type == ROVERCOMMAND_UNDEFINED)
				{
					diag.Diagnostic_Type = DATA_STORAGE;
					diag.Level = ERROR;
					diag.Diagnostic_Message = INITIALIZING_ERROR;
					char tempstr[1024];
					sprintf(tempstr,"File: %s Line Number: %d Command: %s on line: %s Not Supported.",file.c_str(),line_counter,items.at(0).c_str(),line.c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				cmd.command.Command = command_type;
				double v = std::atof(items.at(2).c_str());
				if(v < -0.01)
				{
					cmd.execution_mode = ScriptCommandMode::EXECUTION_COUNT;
					cmd.duration = -1.0;
					cmd.execution_count = (uint16_t)(fabs(v));

				}
				else if (v > 0.01)
				{
					cmd.execution_mode = ScriptCommandMode::DURATION;
					cmd.duration = v;
					cmd.execution_count = 0;
				}
				else
				{
					cmd.execution_mode = ScriptCommandMode::UNTIL_NEXT;
				}
				if((command_type == ROVERCOMMAND_STOPMOVEMENT))  //Parameters: Start Time, Stop Time
				{
					cmd.command_starttime = std::atof(items.at(1).c_str());
					cmd.command.Description = map_RoverCommand_ToString(command_type);

				}
				else if((command_type == ROVERCOMMAND_ARM) ||
						(command_type == ROVERCOMMAND_DISARM))
				{
					cmd.command_starttime = std::atof(items.at(1).c_str());
					cmd.command.Description = map_RoverCommand_ToString(command_type);
				}
				else if((command_type == ROVERCOMMAND_DRIVECOMMAND))  //Parameters: Start Time, Stop Time,Option 1 = Forward Velocity
				{
					cmd.command_starttime = std::atof(items.at(1).c_str());
					json obj;
					obj["ControlType"] = "OpenLoop";
					obj["ControlGroup"] = "ArcadeDrive";
					obj["ForwardVelocityPerc"] = std::atof(items.at(3).c_str());
					obj["RotateZAxisPerc"] = std::atof(items.at(4).c_str());
					cmd.command.CommandText = obj.dump();
					cmd.command.Description = map_RoverCommand_ToString(command_type);
				}
				else
				{
					diag.Diagnostic_Type = DATA_STORAGE;
					diag.Level = ERROR;
					diag.Diagnostic_Message = INITIALIZING_ERROR;
					char tempstr[1024];
					sprintf(tempstr,"File: %s Command: %s Not Supported in Scripting.",file.c_str(),items.at(0).c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				switch(cmd.execution_mode)
				{
				case ScriptCommandMode::DURATION:
					cmd.command_stoptime = cmd.command_starttime+cmd.duration;
					break;
				case ScriptCommandMode::UNTIL_NEXT:
					//Do Nothing;
					break;
				case ScriptCommandMode::EXECUTION_COUNT:
					break;
				}
				if(script_commands.size() >= 1)
				{
					if(script_commands.at(script_commands.size()-1).execution_mode == ScriptCommandMode::UNTIL_NEXT)
					{
						double v = (cmd.command_starttime-.0001);
						script_commands.at(script_commands.size()-1).command_stoptime = v; //Some time that that is much smaller than the loop update period
					}
				}
				cmd.counter = 0;
				script_commands.push_back(cmd);


			}
		}

	}
	script_execution_time = script_commands.at(script_commands.size()-1).command_stoptime;
	command_buffer.push_back(script_commands.at(0).command);
	diag.Diagnostic_Type = DATA_STORAGE;
	diag.Level = NOTICE;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Loaded Script.";
	return diag;
}
void CommandNodeProcess::print_scriptcommand_list()
{
	printf("---Script Command List---\n");
	for(std::size_t i = 0; i < script_commands.size(); ++i)
	{
		printf("[%d] T1: %4.2f T2: %4.2f Command: %s Command Text: %s\n",
				(uint8_t)i,
				script_commands.at(i).command_starttime,
				script_commands.at(i).command_stoptime,
				map_RoverCommand_ToString(script_commands.at(i).command.Command).c_str(),
				script_commands.at(i).command.CommandText.c_str());
	}
}
