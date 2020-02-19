#include "CommandNodeProcess.h"
eros::diagnostic  CommandNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	init_StateList();
	diag = init_PeriodicCommands();
	reset();
	ms_timer = 0;
	gazebo_updaterate = -1.0;
	timeout_value_ms = 0;
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
	gazebo_message_topublish = 0;
	timesince_lastgazeboclock = -1.0;
	diag = update_diagnostic(REMOTE_CONTROL,NOTICE,NOERROR,"No Remote Control Command Yet.");
	diag = update_diagnostic(TARGET_ACQUISITION,INFO,NOERROR,"No Targets Found Yet.");
	return diag;
}
eros::diagnostic CommandNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	if(task_state == TASKSTATE_PAUSE)
	{

	}
	else if(task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
		
	}
	else if(task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	if(task_state == TASKSTATE_RUNNING)
	{
		node_state = NODESTATE_RUNNING;
		current_command.Command = ROVERCOMMAND_NONE;
	}
	if(timesince_lastgazeboclock > 0.0)
	{
		bool simulation_running = false;
		double dt = run_time - timesince_lastgazeboclock;
		if(dt > GAZEBO_PAUSETIME)
		{
			simulation_running = false;
		}
		else
		{
			simulation_running = true;
		}
		for(std::size_t i = 0; i < current_statelist.size(); ++i)
		{
			if(current_statelist.at(i).State == ROVERSTATE_SIMULATION)
			{
				if(simulation_running == true)
				{
					current_statelist.at(i).Option1 = ROVERSTATE_SIMULATION_RUNNING;
					if(gazebo_updaterate < 0.0)
					{
						current_statelist.at(i).StateText = "? Hz";
					}
					else
					{
						char tempstr[128];
						sprintf(tempstr,"%4.2f Hz",gazebo_updaterate);
						current_statelist.at(i).StateText = std::string(tempstr);
					}
				}
				else
				{
					current_statelist.at(i).Option1 = ROVERSTATE_SIMULATION_NOTRUNNING;
				}
			}
		}
	}
	node_state = NODESTATE_RUNNING; //Hack
	batterylevel_perc = 0.0;
	bool temp = true;
	char tempstr[512*(int)(ReadyToArmList.size()+1)];
	memset(tempstr,0,strlen(tempstr));
	if(ReadyToArmList.size() == 0)
	{
		disarmed_reason = "Ready To Arm List Size == 0";
		temp = false;
	}
	
	else
	{
		for(std::size_t i = 0; i < ReadyToArmList.size();++i)
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
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
		}
		disarmed_reason = "None";
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
	if(task_state == TASKSTATE_RUNNING)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
	}
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");
	}
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
eros::diagnostic CommandNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> CommandNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
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
	else if(t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if(node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if(v == false)
			{
				diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
					"Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if(task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE,NOTICE,DIAGNOSTIC_PASSED,
					"Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}

		}
	}
	return diaglist;
}
std::vector<eros::diagnostic> CommandNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED,"Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
eros::diagnostic CommandNodeProcess::init_readytoarm_list(std::vector<std::string> topics)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < topics.size();i++)
	{
		ReadyToArm newarm;
		newarm.topic = topics.at(i);
		newarm.ready_to_arm = false;
		newarm.time_since_lastrx = 0.0;
		ReadyToArmList.push_back(newarm);
	}
	diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Initialized Ready To Arm List.");
	return diag;
}
void CommandNodeProcess::new_readytoarmmsg(std::string topic, bool value)
{
	for(std::size_t i = 0; i < ReadyToArmList.size();i++)
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
	eros::diagnostic diag = root_diagnostic;
	if((msg->Command == ROVERCOMMAND_ARM) || (msg->Command == ROVERCOMMAND_DISARM))
	{
		if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
		{
			diag = update_diagnostic(REMOTE_CONTROL,ERROR,DIAGNOSTIC_FAILED,"Armed Status is DISARMED AND CANNOT ARM!");
			return diag;
		}
		last_command = current_command;
		current_command = convert_fromptr(msg);
		if(msg->Command == ROVERCOMMAND_ARM)
		{
			armeddisarmed_state = ARMEDSTATUS_ARMED;
			diag = update_diagnostic(REMOTE_CONTROL,NOTICE,ROVERCOMMAND_ARM,"Rover is ARMED");
			disarmed_reason = "None";
			return diag;
		}
		else//msg.Command == ROVERCOMMAND_DISARM
		{
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
			diag = update_diagnostic(REMOTE_CONTROL,NOTICE,ROVERCOMMAND_DISARM,"Rover is DISARMED");
			disarmed_reason = "None";
			return diag;
		}
	}
	else if(msg->Command == ROVERCOMMAND_SIMULATIONCCONTROL)
	{
		gazebo_message_topublish = msg->Option1;
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"User Command: %d Not supported.",msg->Command);
		diag = update_diagnostic(REMOTE_CONTROL,WARN,DIAGNOSTIC_FAILED,std::string(tempstr));
		return diag;
	}
	return diag;
}

eros::diagnostic CommandNodeProcess::get_disarmedreason()
{
	eros::diagnostic diag = root_diagnostic;
	if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
	{
		diag = update_diagnostic(REMOTE_CONTROL,WARN,ROVER_DISARMED,disarmed_reason);
	}
	return diag;
}
eros::diagnostic CommandNodeProcess::new_targetmsg(std::string target)
{
	eros::diagnostic diag = root_diagnostic;
	if(target == "outlet")
	{
		if(node_state == NODESTATE_SEARCHING_FOR_RECHARGE_FACILITY)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY;
			node_state = NODESTATE_ACQUIRING_TARGET;
		}
		char tempstr[512];
		sprintf(tempstr,"Found target: %s",target.c_str());
		diag = update_diagnostic(TARGET_ACQUISITION,INFO,NOERROR,std::string(tempstr));
		return diag;

	}
	else if(target == "unknown")
	{
		diag = update_diagnostic(TARGET_ACQUISITION,INFO,NOERROR,"No Target Found");
		return diag;
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"Found target: %s But not currently supported",target.c_str());
		diag = update_diagnostic(TARGET_ACQUISITION,WARN,DROPPING_PACKETS,std::string(tempstr));
		return diag;
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
void CommandNodeProcess::init_StateList()
{
	{
		eros::system_state state;
		state.State = ROVERSTATE_SIMULATION;
		state.Option1 = ROVERSTATE_SIMULATION_UNDEFINED;
		current_statelist.push_back(state);
	}
}
eros::diagnostic CommandNodeProcess::init_PeriodicCommands()
{
	eros::diagnostic diag = root_diagnostic;
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



	for(std::size_t i = 0; i < commands.size(); i++)
	{
		commands.at(i).lasttime_ran = 0.0;
		commands.at(i).send_me = true;
	}
	periodic_commands = commands;
	diag = update_diagnostic(SOFTWARE,INFO,INITIALIZING, "Initialized Periodic Command List");
	return diag;
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
	eros::diagnostic diag = root_diagnostic;
	std::vector<std::string> files_to_load;
	DIR *dpdf;
	struct dirent *epdf;

	dpdf = opendir(directory.c_str());
	if (dpdf != NULL)
	{
		while ((epdf = (readdir(dpdf))))
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
					char tempstr[1024];
					sprintf(tempstr,"File: %s Line Number: %d Command: %s on line: %s Not Supported.",file.c_str(),line_counter,items.at(0).c_str(),line.c_str());
					diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
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
					char tempstr[1024];
					sprintf(tempstr,"File: %s Command: %s Not Supported in Scripting.",file.c_str(),items.at(0).c_str());
					diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
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
	diag = update_diagnostic(DATA_STORAGE,NOTICE,NOERROR, "Loaded Script.");
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
