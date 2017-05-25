#include "command_node_process.h"

CommandNodeProcess::CommandNodeProcess()
{
	ms_timer = 0;
	timeout_value_ms = 0;
	gettimeofday(&init_time,NULL);
    armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    ReadyToArmList.clear();
    readytoarm = false;
    batterylevel_perc = 0.0;
    run_time = 0.0;
	current_command.Command = ROVERCOMMAND_BOOT;
	last_command = current_command;
	node_state = NODESTATE_BOOTING;
    //armedcommand = ARMEDCOMMAND_DISARM;
}
CommandNodeProcess::~CommandNodeProcess()
{

}
icarus_rover_v2::diagnostic CommandNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;

	return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::init_readytoarm_list(std::vector<std::string> topics)
{
    for(int i = 0; i < topics.size();i++)
    {
        ReadyToArm newarm;
        newarm.topic = topics.at(i);
        newarm.ready_to_arm = false;
        ReadyToArmList.push_back(newarm);
    }
    diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Initialized Ready To Arm List.";
	mylogger->log_info(std::string(diagnostic.Description));
    return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::new_readytoarmmsg(std::string topic, bool value)
{
	//printf("Got a topic: %s\n",topic.c_str());
	bool ready_to_arm_check = true;
    for(int i = 0; i < ReadyToArmList.size();i++)
    {
        if(ReadyToArmList.at(i).topic == topic)
        {
        	//printf("Matched topic: %s\n",topic.c_str());
            ReadyToArmList.at(i).ready_to_arm = value;
            if(value == false)
            {
            	ready_to_arm_check = false;
            }
        }
    }
    for(int i = 0; i < ReadyToArmList.size();i++)
    {
    	if(ReadyToArmList.at(i).ready_to_arm == false)
    	{
    		//printf("Topic: %s Reports is unable to Arm.\n",ReadyToArmList.at(i).topic.c_str());
    		ready_to_arm_check = false;
    	}
    }
    if((ReadyToArmList.size() > 0) and (ready_to_arm_check == true))
    {
    	readytoarm = true;
        diagnostic.Diagnostic_Type = REMOTE_CONTROL;
        diagnostic.Level = INFO;
        diagnostic.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Topic: %s Reports is Ready To Arm",topic.c_str());
        diagnostic.Description = std::string(tempstr);
    }
    else
    {
    	readytoarm = false;
        diagnostic.Diagnostic_Type = REMOTE_CONTROL;
        diagnostic.Level = INFO; //WARN;
        diagnostic.Diagnostic_Message = ROVER_DISARMED;
        char tempstr[512];
        sprintf(tempstr,"Topic: %s Reports is Unable To Arm",topic.c_str());
        diagnostic.Description = std::string(tempstr);
    }
   
    return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::new_user_armcommandmsg(uint8_t value)
{
	if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
	{
		diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		diagnostic.Level = FATAL;
		diagnostic.Description = "Armed Status is UNDEFINED!";
		diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
		return diagnostic;
	}
	else if(armeddisarmed_state == ARMEDSTATUS_DISARMED)
	{
		if(value == ROVERCOMMAND_ARM)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_ARM;
			current_command.Option1 = 0;
			current_command.Option2 = 0;
			current_command.Option3 = 0;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = NOTICE;
			diagnostic.Description = "Rover is ARMED";
			diagnostic.Diagnostic_Message = ROVERCOMMAND_ARM;
			armeddisarmed_state = ARMEDSTATUS_ARMED;
			return diagnostic;
		}
		else if(value == ROVERCOMMAND_DISARM)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_DISARM;
			current_command.Option1 = 0;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = INFO;
			diagnostic.Description = "Rover is still DISARMED";
			diagnostic.Diagnostic_Message = ROVERCOMMAND_DISARM;
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
			return diagnostic;
		}
	}
	else if(armeddisarmed_state == ARMEDSTATUS_ARMED)
	{
		if(value == ROVERCOMMAND_DISARM)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_DISARM;
			current_command.Option1 = 0;
			current_command.Option2 = 0;
			current_command.Option3 = 0;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = NOTICE;
			diagnostic.Description = "Rover is DISARMED";
			diagnostic.Diagnostic_Message = ROVERCOMMAND_DISARM;
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
			return diagnostic;
		}
		else if(value == ROVERCOMMAND_ARM)
		{
			last_command = current_command;
			current_command.Command = ROVERCOMMAND_ARM;
			current_command.Option1 = 0;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = INFO;
			diagnostic.Description = "Rover is still ARMED";
			diagnostic.Diagnostic_Message = ROVERCOMMAND_ARM;
			armeddisarmed_state = ARMEDSTATUS_ARMED;
			return diagnostic;
		}
	}

	/*
	if(armeddisarmed_state == ARMEDSTATUS_UNDEFINED)
	{
		diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		diagnostic.Level = FATAL;
		diagnostic.Description = "Armed Status is UNDEFINED!";
		diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
		return diagnostic;
	}
	else if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
	{
		diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		diagnostic.Level = WARN;
		diagnostic.Description = "Cannot set new Arm Command";
		diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
		return diagnostic;
	}
	else if(armeddisarmed_state == ARMEDSTATUS_DISARMED)
	{
		if(value ==)
		{
			//armedcommand = ARMEDCOMMAND_ARM;
			armeddisarmed_state = ARMEDSTATUS_ARMED;
			c
		}
	}
	else if(armeddisarmed_state == ARMEDSTATUS_ARMED)
	{
		if(value == ARMEDCOMMAND_DISARM)
		{
			//armedcommand = ARMEDCOMMAND_DISARM;
			armeddisarmed_state = ARMEDSTATUS_DISARMED;
			diagnostic.Diagnostic_Type = REMOTE_CONTROL;
			diagnostic.Level = NOTICE;
			diagnostic.Description = "Rover is DISARMED";
			diagnostic.Diagnostic_Message = ROVER_DISARMED;
			return diagnostic;
		}
	}
	*/
	diagnostic.Diagnostic_Type = REMOTE_CONTROL;
	diagnostic.Level = WARN;
	diagnostic.Description = "An Unknown Problem occurred.";
	diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
	return diagnostic;

}
icarus_rover_v2::diagnostic CommandNodeProcess::update(double dt)
{
	if(node_state == NODESTATE_RUNNING)
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

    bool temp = true;
    for(int i = 0; i < ReadyToArmList.size();i++)
    {
        if(ReadyToArmList.at(i).ready_to_arm == false)
        {
            temp = false;
            char tempstr[255];
            sprintf(tempstr,"Topic: %s Reports is Unable to Arm.",ReadyToArmList.at(i).topic.c_str());
            mylogger->log_warn(tempstr);
        }
    }
    if(ReadyToArmList.size() == 0)
    {
    	readytoarm = false;
    }
    else
    {
    	readytoarm = temp;
    }
    if(readytoarm == false)
    {
        armeddisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
        //armedcommand = ARMEDCOMMAND_DISARM;
    }
    else if(readytoarm == true)
    {

    	if(armeddisarmed_state == ARMEDSTATUS_DISARMED_CANNOTARM)
    	{

    		armeddisarmed_state = ARMEDSTATUS_DISARMED;
    	}
    }
    run_time += dt;
	
    if(current_command.Command == ROVERCOMMAND_NONE)
    {
    	for(int i = 0; i < periodic_commands.size(); i++)
    	{
    		double time_to_run = periodic_commands.at(i).lasttime_ran + (1.0/periodic_commands.at(i).rate_hz);
    		if(time_to_run <= run_time)
    		{
    			current_command = periodic_commands.at(i).command;
    			periodic_commands.at(i).lasttime_ran = run_time;
    		}
    	}
    }
	return diagnostic;
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
icarus_rover_v2::diagnostic CommandNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;
		node_state = NODESTATE_RUNNING;
		current_command.Command = ROVERCOMMAND_NONE;
		all_device_info_received = true;
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::init_PeriodicCommands(std::vector<PeriodicCommand> commands)
{
	for(int i = 0; i < commands.size(); i++)
	{
		commands.at(i).lasttime_ran = 0.0;
	}
	periodic_commands = commands;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Initialized Periodic Command List";
	mylogger->log_info(std::string(diagnostic.Description));
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
