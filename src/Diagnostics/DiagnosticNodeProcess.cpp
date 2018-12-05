#include "DiagnosticNodeProcess.h"
icarus_rover_v2::diagnostic  DiagnosticNodeProcess::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	current_command.Command = ROVERCOMMAND_NONE;
	last_cmd_timer = 0.0;
	last_cmddiagnostic_timer = 0.0;
	lcd_partnumber = "617003";
	lcd_width = 20;
	lcd_height = 4;
	lcd_clock = 0;
	battery_voltage = 0.0;
	voltage_received = false;
	armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	bad_diagnostic_received = false;
	lcd_available = true;
	lcdclock_timer = 0.0;
	RCControl = true;
	init_diaglevels();
	return diagnostic;
}
icarus_rover_v2::diagnostic DiagnosticNodeProcess::update(double t_dt,double t_ros_time)
{
	last_cmddiagnostic_timer += t_dt;
	lcdclock_timer+=t_dt;

	icarus_rover_v2::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	if(lcdclock_timer > 0.5)
	{
		int v = lcd_clock;
		v++;
		if(v > 7)
		{
			v = 0;
		}
		lcd_clock = v;
		lcdclock_timer = 0.0;
	}
	if(lcd_available == false)
	{
		if(initialized == true){ ready = true; }
	}
	bool status = true;
	for(std::size_t i = 0; i < diaglevels.size(); i++)
	{
		diaglevels.at(i).last_time += t_dt;

	}
	if(last_cmddiagnostic_timer > 30.0)
	{
		status = false;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[512];
		sprintf(tempstr,"Have not received CMD:DIAGNOSTIC in: %f Seconds",last_cmddiagnostic_timer);
		diag.Description = std::string(tempstr);
	}
	if(status == true)
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running";
	}

	diagnostic = diag;
	return diag;
}
icarus_rover_v2::diagnostic DiagnosticNodeProcess::new_devicemsg(const icarus_rover_v2::device::ConstPtr& device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(device->DeviceName == host_name)
	{

	}
	else if(device->DeviceType == "LCD")
	{
		if(device->PartNumber == "617003")
		{
			lcd_height = 4;
			lcd_width = 20;
			ready = true;
		}
		else
		{
			diag.Level = ERROR;
			diag.Diagnostic_Type = DATA_STORAGE;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			char tempstr[512];
			sprintf(tempstr,"Unsupported Device: %s\n",device->PartNumber.c_str());
			diag.Description = std::string(tempstr);
		}
	}
	else
	{
		diag.Level = ERROR;
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		char tempstr[512];
		sprintf(tempstr,"Unsupported Device Message: %s\n",device->DeviceName.c_str());
		diag.Description = std::string(tempstr);
	}
	return diag;
}
std::vector<icarus_rover_v2::diagnostic> DiagnosticNodeProcess::new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		RCControl = true;
		last_cmddiagnostic_timer = 0.0;
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

	else if(t_msg->Command == ROVERCOMMAND_ACQUIRE_TARGET)
	{
		RCControl = false;
	}
	else if(t_msg->Command == ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY)
	{
		RCControl = false;
	}
	else if(t_msg->Command == ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY)
	{
		RCControl = false;
	}
	else
	{
		RCControl = true;
	}

	return diaglist;
}
std::vector<icarus_rover_v2::diagnostic> DiagnosticNodeProcess::check_programvariables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
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
std::string DiagnosticNodeProcess::build_lcdmessage()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(lcd_partnumber == "617003")
	{
		char buffer[lcd_width*lcd_height];
		//printf("a: %s\n",get_armedstatestr(armed_state).c_str());
		//printf("b: %s\n",get_batteryvoltagestr().c_str());
		//printf("c: %s\n",get_batterylevelstr(battery_level).c_str());
		sprintf(buffer,"%s%s%s",get_armedstatestr(armed_state).c_str(),get_batteryvoltagestr().c_str(),get_batterylevelstr(battery_level).c_str());
		//printf("1: %s\n",buffer);
		std::string diag_str = get_diagstr();
		//printf("2: %s\n",diag_str.c_str());
		//printf("3: %d\n",bad_diagnostic_received);
		if(bad_diagnostic_received == false)
		{
			sprintf(buffer,"%s%s",buffer,diag_str.c_str());
			sprintf(buffer,"%s%s",buffer,get_lcdcommandstr().c_str());
			sprintf(buffer,"%s                   %c",buffer,get_lcdclockchar(lcd_clock));
		}
		else
		{
			sprintf(buffer,"%s%s%c",buffer,diag_str.c_str(),get_lcdclockchar(lcd_clock));
		}
		return std::string(buffer);

	}
	else
	{
		return "";
	}
}
std::string DiagnosticNodeProcess::get_batterylevelstr(double v)
{
	if(v >= 75.0)
	{
		return ">>>>";
	}
	else if(v >= 50.0)
	{
		return ">>>-";
	}
	else if(v >= 25.0)
	{
		return ">>--";
	}
	else if(v > 5.0)
	{
		return ">---";
	}
	else
	{
		return "----";
	}
}
unsigned char DiagnosticNodeProcess::get_lcdclockchar(int v)
{
	switch(v)
	{
	case 0: return '-'; break;
	case 1: return 92; break;
	case 2: return '!'; break;
	case 3: return '/'; break;
	case 4: return '-'; break;
	case 5: return 92; break;
	case 6: return '!'; break;
	case 7: return '/'; break;
	default: return 'X'; break;
	}
}
std::string DiagnosticNodeProcess::get_batteryvoltagestr()
{
	if(voltage_received == false)
	{
		return "--.-V";
	}
	else
	{
		char tempstr[5];
		if(battery_voltage < 10.0)
		{
			sprintf(tempstr," %2.1fV",battery_voltage);
		}
		else
		{
			sprintf(tempstr,"%2.1fV",battery_voltage);
		}

		return std::string(tempstr);
	}
}
std::string DiagnosticNodeProcess::get_armedstatestr(uint8_t v)
{
	switch(v)
	{
	case ARMEDSTATUS_UNDEFINED: return "UNDEFINED  "; break;
	case ARMEDSTATUS_ARMED: return "ARMED      "; break;
	case ARMEDSTATUS_DISARMED_CANNOTARM: return "CANNOT ARM "; break;
	case ARMEDSTATUS_DISARMED: return "DISARMED   "; break;
	case ARMEDSTATUS_DISARMING: return "DISARMING  "; break;
	case ARMEDSTATUS_ARMING: return "ARMING     "; break;
	default: return "UNDEFINED  "; break;
	}

}
std::string DiagnosticNodeProcess::get_diagstr()
{
	std::string device,level,message,desc;
	bool found_one = false;
	if(any_diagnostic_received == false)
	{
		return "NO DIAG RECEIVED    ";
	}
	icarus_rover_v2::diagnostic worst_diag;
	for(std::size_t i = diaglevels.size()-1; i >0; i--)
	{
		if(diaglevels.at(i).last_time < WORSTDIAG_TIMELIMIT)
		{
			worst_diag = diaglevels.at(i).diag;
			found_one = true;

		}
	}
	bad_diagnostic_received = found_one;
	if(found_one == false)
	{
		return "NO ERROR            ";
	}

	if(worst_diag.DeviceName.length() > 6)
	{
		device = worst_diag.DeviceName.substr(0,6);
	}
	else
	{
		device = worst_diag.DeviceName;
		device.append(6 - device.length(), ' ');
	}
	switch(worst_diag.Level)
	{
	case DEBUG: level = "DEBUG"; break;
	case INFO: level = "INFO "; break;
	case NOTICE: level = "NTICE"; break;
	case WARN: level = "WARN "; break;
	case ERROR: level = "ERROR"; break;
	case FATAL: level = "FATAL"; break;
	default: level = "UNDFD"; break;
	}
	switch(worst_diag.Diagnostic_Message)
	{
	case NOERROR: message = "NOERROR"; break;
	case INITIALIZING: message = "INITLZG"; break;
	case INITIALIZING_ERROR: message = "INITERR"; break;
	case DROPPING_PACKETS: message = "DROPPKT"; break;
	case MISSING_HEARTBEATS: message = "MSNGHBT"; break;
	case DEVICE_NOT_AVAILABLE: message = "DEV N/A"; break;
	case ROVER_ARMED: message = "  ARMED"; break;
	case ROVER_DISARMED: message = " DISARM"; break;
	case TEMPERATURE_HIGH: message = "HIGHTMP"; break;
	case TEMPERATURE_LOW: message = " LOWTMP"; break;
	case DIAGNOSTIC_PASSED: message = "DIAGPSD"; break;
	case DIAGNOSTIC_FAILED: message = "DIAGFLD"; break;
	case RESOURCE_LEAK: message = "RESLEAK"; break;
	case HIGH_RESOURCE_USAGE: message = "RESHIGH"; break;
	case UNKNOWN_STATE: message = "UNKSTAT"; break;
	case UNKNOWN_MESSAGE: message = "UNKNOWN"; break;
	}
	if(worst_diag.Description.length() > 39)
	{
		desc = worst_diag.Description.substr(0,39);
	}
	else
	{
		desc = worst_diag.Description;
		desc.append(39 - desc.length(), ' ');
	}
	char tempstr[lcd_width*lcd_height];
	sprintf(tempstr,"%s %s %s%s",device.c_str(),level.c_str(),message.c_str(),desc.c_str());

	return std::string(tempstr);

}
std::string DiagnosticNodeProcess::get_lcdcommandstr()
{
	std::string tempstr = "";
	if(last_cmd_timer < 0.5)
	{
		switch(current_command.Command)
		{
		case ROVERCOMMAND_BOOT:
			tempstr = "CMD:BOOTING"; break;
		case ROVERCOMMAND_NONE:
			tempstr = "CMD:NONE"; break;
		case ROVERCOMMAND_RUNDIAGNOSTIC:
			if(current_command.Option1 == LEVEL1)
			{
				tempstr = "";
			}
			else
			{
				char tempstr2[20];
				sprintf(tempstr2,"CMD:RUN DIAG TEST:%d",current_command.Option1);
				tempstr = std::string(tempstr2);
			}
			break;
		case ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY:
			tempstr = "CMD:SEARCH RECHARGE"; break;
		case ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY:
			tempstr = "CMD:STOP SRCH RCHRG"; break;
		case ROVERCOMMAND_ACQUIRE_TARGET:
			tempstr = "CMD:ACQUIRE TARGET"; break;
		case ROVERCOMMAND_ARM:
			tempstr = "CMD:ARM"; break;
		case ROVERCOMMAND_DISARM:
			tempstr = "CMD:DISARM"; break;
		case ROVERCOMMAND_CONFIGURE:
			tempstr = "CMD:CONFIGURE"; break;
		case ROVERCOMMAND_RUN:
			tempstr = "CMD:RUN"; break;
		default:
			char tempstr2[20];
			sprintf(tempstr2,"CMD: UNKNOWN (%d)",current_command.Command);
			tempstr = std::string(tempstr2); break;

		}
	}
	else
	{

	}
	tempstr.append(19 - tempstr.length(), ' ');
	return tempstr;
}
void DiagnosticNodeProcess::add_Task(Task v)
{
	v.CPU_Perc = 0;
	v.PID = -1;
	v.RAM_MB = 0;
	v.last_diagnostic_received = 0;
	v.last_heartbeat_received = 0;
	v.last_resource_received = 0;
	TaskList.push_back(v);
}
void DiagnosticNodeProcess::new_heartbeatmsg(std::string topicname)
{
	for(int i = 0; i < TaskList.size(); i++)
	{
		if(	TaskList.at(i).heartbeat_topic == topicname)
		{
			TaskList.at(i).last_heartbeat_received = run_time;
		}
	}
}
void DiagnosticNodeProcess::new_resourcemsg(std::string topicname,const icarus_rover_v2::resource::ConstPtr& resource)
{
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).resource_topic == topicname)
		{
			TaskList.at(i).last_resource_received = run_time;
			TaskList.at(i).CPU_Perc = resource->CPU_Perc;
			TaskList.at(i).RAM_MB = resource->RAM_MB;
			TaskList.at(i).PID = resource->PID;
		}
	}

	std::size_t resource_available_topic = topicname.find("resource_available");
	if(resource_available_topic != std::string::npos)
	{
		bool found = true;
		for(std::size_t i = 0; i < DeviceResourceAvailableList.size();i++)
		{
			if(DeviceResourceAvailableList.at(i).Device_Name == resource->Node_Name)
			{
				found = false;
				DeviceResourceAvailableList.at(i).CPU_Perc_Available = resource->CPU_Perc;
				DeviceResourceAvailableList.at(i).RAM_Mb_Available = resource->RAM_MB;
				break;
			}
		}
		if(found == true)
		{
			DeviceResourceAvailable newdevice;
			newdevice.Device_Name = resource->Node_Name;
			newdevice.CPU_Perc_Available = resource->CPU_Perc;
			newdevice.RAM_Mb_Available = resource->RAM_MB;
			DeviceResourceAvailableList.push_back(newdevice);
		}
	}
}
void DiagnosticNodeProcess::new_diagnosticmsg(std::string topicname,const icarus_rover_v2::diagnostic::ConstPtr& diagnostic)
{
	bool add_me = true;
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).diagnostic_topic == topicname)
		{
			TaskList.at(i).last_diagnostic_received = run_time;
			TaskList.at(i).last_diagnostic_level = diagnostic->Level;
		}
	}
	if(diagnostic->Level >= WARN)
	{
		for(std::size_t i = 0; i < diaglevels.size(); i++)
		{
			if(diaglevels.at(i).Level == diagnostic->Level)
			{
				diaglevels.at(i).diag = convert_fromptr(diagnostic);
				diaglevels.at(i).last_time = 0.0;
			}
		}
	}
	any_diagnostic_received = true;
}
std::vector<icarus_rover_v2::diagnostic> DiagnosticNodeProcess::check_tasks()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	int task_ok_counter = 0;
	for(int i = 0; i < TaskList.size(); i++)
	{
		bool task_ok = true;
		DiagnosticNodeProcess::Task Task = TaskList.at(i);
		if(Task.CPU_Perc > CPU_usage_threshold_percent)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s is using high CPU resource: %d/%d %%",
					Task.Task_Name.c_str(),Task.CPU_Perc,CPU_usage_threshold_percent);
			diag.Diagnostic_Type = SOFTWARE;
			diag.Diagnostic_Message = HIGH_RESOURCE_USAGE;
			diag.Level = WARN;
			diag.Description = tempstr;
			diaglist.push_back(diag);
		}
		if(Task.RAM_MB > RAM_usage_threshold_MB)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s is using high RAM resource: %ld/%d (MB)",
					Task.Task_Name.c_str(),Task.RAM_MB,RAM_usage_threshold_MB);
			diag.Diagnostic_Type = SOFTWARE;
			diag.Diagnostic_Message = HIGH_RESOURCE_USAGE;
			diag.Level = WARN;
			diag.Description = tempstr;
			diaglist.push_back(diag);
		}
		
		double heartbeat_time_duration = run_time-Task.last_heartbeat_received;
		if(heartbeat_time_duration > 5.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported heartbeats in %.1f seconds",
					Task.Task_Name.c_str(),heartbeat_time_duration);
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Diagnostic_Message = MISSING_HEARTBEATS;
			diag.Level = FATAL;
			diag.Description = tempstr;
			diaglist.push_back(diag);
		}

		if(task_ok == true){task_ok_counter++;}
	}
	if(task_ok_counter == TaskList.size())
	{
		if(TaskList.size() > 0)
		{
			if(ready == true) { ready_to_arm = true; }
			char tempstr[255];
			sprintf(tempstr,"%d/%d (All) Tasks Operational.",task_ok_counter,(int)TaskList.size());
			icarus_rover_v2::diagnostic system_diag;
			system_diag.Node_Name = node_name;
			system_diag.System = ROVER;
			system_diag.SubSystem = ENTIRE_SUBSYSTEM;
			system_diag.Component = DIAGNOSTIC_NODE;
			system_diag.Diagnostic_Message = NOERROR;
			system_diag.Diagnostic_Type = NOERROR;
			system_diag.Level = NOTICE;
			system_diag.Description = std::string(tempstr);
			diaglist.push_back(system_diag);
		}
		else
		{
			ready_to_arm = false;
			diag.Diagnostic_Type = SOFTWARE;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			diag.Level = FATAL;
			diag.Description = "Not listening to Any Tasks.";
			diaglist.push_back(diag);
		}

	}
	else
	{
		ready_to_arm = false;
		char tempstr[255];
		sprintf(tempstr,"%d/%d Tasks are in WARN state or Higher!",(int)TaskList.size()-task_ok_counter,(int)TaskList.size());
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diag.Level = WARN;
		diag.Description = tempstr;
		diaglist.push_back(diag);
	}
	return diaglist;
}
void DiagnosticNodeProcess::init_diaglevels()
{
	{
		DiagLevel diaglev;
		diaglev.Level = WARN;
		icarus_rover_v2::diagnostic diag;
		diag.Level = diaglev.Level;
		diaglev.diag = diag;
		diaglev.last_time = WORSTDIAG_TIMELIMIT*2.0;
		diaglevels.push_back(diaglev);
	}
	{
		DiagLevel diaglev;
		diaglev.Level = ERROR;
		icarus_rover_v2::diagnostic diag;
		diag.Level = diaglev.Level;
		diaglev.diag = diag;
		diaglev.last_time = WORSTDIAG_TIMELIMIT*2.0;
		diaglevels.push_back(diaglev);
	}
	{
		DiagLevel diaglev;
		diaglev.Level = FATAL;
		icarus_rover_v2::diagnostic diag;
		diag.Level = diaglev.Level;
		diaglev.diag = diag;
		diaglev.last_time = WORSTDIAG_TIMELIMIT*2.0;
		diaglevels.push_back(diaglev);
	}
}
