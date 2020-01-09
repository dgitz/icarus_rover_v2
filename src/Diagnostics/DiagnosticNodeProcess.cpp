#include "DiagnosticNodeProcess.h"
eros::diagnostic  DiagnosticNodeProcess::finish_initialization()
{
	supported_partnumbers.push_back(PN_617003);
	eros::diagnostic diag = root_diagnostic;
	current_command.Command = ROVERCOMMAND_NONE;
	last_cmd_timer = 0.0;
	last_cmddiagnostic_timer = 0.0;
	lcd_partnumber = PN_617003;
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
	init_subsystemdiagnostics();
	return diag;
}
eros::diagnostic DiagnosticNodeProcess::update(double t_dt,double t_ros_time)
{
	last_cmddiagnostic_timer += t_dt;
	lcdclock_timer+=t_dt;

	eros::diagnostic diag = root_diagnostic;
		if(initialized == false)
	{
		diag = update_diagnostic(REMOTE_CONTROL,NOTICE,NOERROR,"No Remote Control Command Yet.");
		diag = update_diagnostic(DATA_STORAGE,NOTICE,INITIALIZING,"Initializing.");
	}

	diag = update_baseprocess(t_dt,t_ros_time);
	diag = update_diagnostic(diag);
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
		char tempstr[512];
		sprintf(tempstr,"Have not received CMD:DIAGNOSTIC in: %f Seconds",last_cmddiagnostic_timer);
		diag = update_diagnostic(COMMUNICATIONS,WARN,DROPPING_PACKETS,std::string(tempstr));
	}
	for(std::size_t i = 0; i < subsystem_diagnostics.size(); ++i)
	{
		if(subsystem_diagnostics.at(i).diagnostics.size() > 0)
		{
			uint8_t highest_level = 0;
			for(std::size_t j = 0; j < subsystem_diagnostics.at(i).diagnostics.size(); ++j)
			{
				if(subsystem_diagnostics.at(i).diagnostics.at(j).Level > highest_level)
				{
					highest_level = subsystem_diagnostics.at(i).diagnostics.at(j).Level;
				}
			}
			subsystem_diagnostics.at(i).Level = highest_level;
		}
		else
		{
			subsystem_diagnostics.at(i).Level = LEVEL_UNKNOWN;
		}
	}
	if(status == true)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running");
		if((is_initialized() == true) and (is_ready() == true))
		{
			update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error");
		}
	}
	return diag;
}
eros::diagnostic DiagnosticNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	if(device->DeviceName == host_name)
	{

	}
	else if(device->DeviceType == DEVICETYPE_LCD)
	{
		if(device->PartNumber == PN_617003)
		{
			lcd_height = 4;
			lcd_width = 20;
			diag = update_diagnostic(REMOTE_CONTROL,INFO,NOERROR,"LCD Initialized.");
			ready = true;
		}
		else
		{
			char tempstr[512];
			sprintf(tempstr,"Unsupported Device: %s\n",device->PartNumber.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
		}
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"Unsupported Device Message: %s\n",device->DeviceName.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	}
	return diag;
}
std::vector<eros::diagnostic> DiagnosticNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
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
std::vector<eros::diagnostic> DiagnosticNodeProcess::check_programvariables()
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
std::string DiagnosticNodeProcess::build_lcdmessage()
{
	if(lcd_partnumber == PN_617003)
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
	eros::diagnostic worst_diag;
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
	for(std::size_t i = 0; i < TaskList.size(); i++)
	{
		if(	TaskList.at(i).heartbeat_topic == topicname)
		{
			TaskList.at(i).last_heartbeat_received = run_time;
		}
	}
	update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"Heartbeat Updated.");
}
void DiagnosticNodeProcess::new_resourcemsg(std::string topicname,const eros::resource::ConstPtr& resource)
{
	for(std::size_t i = 0; i < TaskList.size();i++)
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
void DiagnosticNodeProcess::new_diagnosticmsg(std::string topicname,const eros::diagnostic::ConstPtr& diagnostic)
{
	for(std::size_t i = 0; i < TaskList.size();i++)
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
	for(std::size_t i = 0; i < subsystem_diagnostics.size(); ++i)
	{
		if(diagnostic->Diagnostic_Type == subsystem_diagnostics.at(i).Diagnostic_Type)
		{
			bool add_me = true;
			for(std::size_t j = 0; j < subsystem_diagnostics.at(i).diagnostics.size(); ++j)
			{
				if((subsystem_diagnostics.at(i).diagnostics.at(j).DeviceName == diagnostic->DeviceName) and 
				   (subsystem_diagnostics.at(i).diagnostics.at(j).Node_Name == diagnostic->Node_Name))
				{
					add_me = false;
					subsystem_diagnostics.at(i).diagnostics.at(j) = convert_fromptr(diagnostic);
					if(diagnostic->Level <= FATAL)
					{
						subsystem_diagnostics.at(i).level_counters.at(diagnostic->Level)++;
					}
				}
			}
			if(add_me == true)
			{
				subsystem_diagnostics.at(i).diagnostics.push_back(convert_fromptr(diagnostic));
				if(diagnostic->Level <= FATAL)
					{
						subsystem_diagnostics.at(i).level_counters.at(diagnostic->Level)++;
					}
			}
		}
	}
	any_diagnostic_received = true;
}
std::vector<eros::diagnostic> DiagnosticNodeProcess::check_tasks()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	std::size_t task_ok_counter = 0;
	for(std::size_t i = 0; i < TaskList.size(); i++)
	{
		bool task_ok = true;
		DiagnosticNodeProcess::Task Task = TaskList.at(i);
		if(Task.CPU_Perc > CPU_usage_threshold_percent)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s is using high CPU resource: %d/%d %%",
					Task.Task_Name.c_str(),Task.CPU_Perc,CPU_usage_threshold_percent);
			diag = update_diagnostic(SYSTEM_RESOURCE,WARN,HIGH_RESOURCE_USAGE,std::string(tempstr));
			diaglist.push_back(diag);
		}
		if(Task.RAM_MB > RAM_usage_threshold_MB)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s is using high RAM resource: %ld/%d (MB)",
					Task.Task_Name.c_str(),Task.RAM_MB,RAM_usage_threshold_MB);
			diag = update_diagnostic(SYSTEM_RESOURCE,WARN,HIGH_RESOURCE_USAGE,std::string(tempstr));
			diaglist.push_back(diag);
		}
		
		double heartbeat_time_duration = run_time-Task.last_heartbeat_received;
		if(heartbeat_time_duration > 5.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported heartbeats in %.1f seconds",
					Task.Task_Name.c_str(),heartbeat_time_duration);
			diag = update_diagnostic(COMMUNICATIONS,ERROR,MISSING_HEARTBEATS,std::string(tempstr));
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
			sprintf(tempstr,"%d/%d (All) Tasks Operational.",(int)task_ok_counter,(int)TaskList.size());
			diag = update_diagnostic(SOFTWARE,INFO,NOERROR,std::string(tempstr));
			diaglist.push_back(diag);
		}
		else
		{
			ready_to_arm = false;
			diag = update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,"Not listening to Any Tasks.");
			diaglist.push_back(diag);
		}

	}
	else
	{
		ready_to_arm = false;
		char tempstr[255];
		sprintf(tempstr,"%d/%d Tasks are in WARN state or Higher!",(int)TaskList.size()-(int)task_ok_counter,(int)TaskList.size());
		diag = update_diagnostic(SOFTWARE,WARN,DEVICE_NOT_AVAILABLE,std::string(tempstr));
		diaglist.push_back(diag);
	}
	return diaglist;
}
void DiagnosticNodeProcess::init_diaglevels()
{
	{
		DiagLevel diaglev;
		diaglev.Level = WARN;
		eros::diagnostic diag;
		diag.Level = diaglev.Level;
		diaglev.diag = diag;
		diaglev.last_time = WORSTDIAG_TIMELIMIT*2.0;
		diaglevels.push_back(diaglev);
	}
	{
		DiagLevel diaglev;
		diaglev.Level = ERROR;
		eros::diagnostic diag;
		diag.Level = diaglev.Level;
		diaglev.diag = diag;
		diaglev.last_time = WORSTDIAG_TIMELIMIT*2.0;
		diaglevels.push_back(diaglev);
	}
	{
		DiagLevel diaglev;
		diaglev.Level = FATAL;
		eros::diagnostic diag;
		diag.Level = diaglev.Level;
		diaglev.diag = diag;
		diaglev.last_time = WORSTDIAG_TIMELIMIT*2.0;
		diaglevels.push_back(diaglev);
	}
}
void DiagnosticNodeProcess::init_subsystemdiagnostics()
{
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = ELECTRICAL;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = SOFTWARE;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = COMMUNICATIONS;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = SENSORS;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = ACTUATORS;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = DATA_STORAGE;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = REMOTE_CONTROL;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = TARGET_ACQUISITION;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = POSE;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = TIMING;
		subsystem_diagnostics.push_back(diag);		
	}
	{
		SubSystemDiagnostic diag;
		diag.Diagnostic_Type = SYSTEM_RESOURCE;
		subsystem_diagnostics.push_back(diag);		
	}

	
	for(std::size_t i = 0; i < subsystem_diagnostics.size(); ++i)
	{
		subsystem_diagnostics.at(i).Level = LEVEL_UNKNOWN;
		for(int j = 0; j <= FATAL; ++j)
		{
			subsystem_diagnostics.at(i).level_counters.push_back(0);
		}
		eros_subsystem_diagnostic.Diagnostic_Type.push_back(subsystem_diagnostics.at(i).Diagnostic_Type);
		eros_subsystem_diagnostic.Level.push_back(LEVEL_UNKNOWN);
	}
}
DiagnosticNodeProcess::SubSystemDiagnostic DiagnosticNodeProcess::get_subsystem_diagnostic(uint8_t Diagnostic_Type)
{
	for(std::size_t i = 0; i < subsystem_diagnostics.size(); ++i)
	{
		if(subsystem_diagnostics.at(i).Diagnostic_Type == Diagnostic_Type)
		{
			return subsystem_diagnostics.at(i);
		}
	}
	SubSystemDiagnostic diag;
	diag.Diagnostic_Type = GENERAL_ERROR;
	return diag;
}
std::string DiagnosticNodeProcess::print_subsystem_diagnostics()
{
	std::string tempstr = "\n--- SUBSYSTEM DIAGNOSTICS ---\n";
	for(std::size_t i = 0; i < subsystem_diagnostics.size(); ++i)
	{
		tempstr += "[" + std::to_string(i) + "] Type: ";
		tempstr += diagnostic_helper.get_DiagTypeString(subsystem_diagnostics.at(i).Diagnostic_Type);
		tempstr += " Level: ";
		tempstr += diagnostic_helper.get_DiagLevelString(subsystem_diagnostics.at(i).Level);
		tempstr += " ";
		for(uint8_t j = 0; j <= FATAL; ++j)
		{
			tempstr += "L";
			tempstr += std::to_string(j);
			tempstr += ":";
			tempstr += std::to_string(subsystem_diagnostics.at(i).level_counters.at(j));
			tempstr += " ";
		}
		tempstr += "\n";
		for(std::size_t j = 0; j < subsystem_diagnostics.at(i).diagnostics.size(); ++j)
		{
			tempstr += "\tNode: ";
			tempstr += subsystem_diagnostics.at(i).diagnostics.at(j).Node_Name;
			tempstr += " Device: ";
			tempstr += subsystem_diagnostics.at(i).diagnostics.at(j).DeviceName;
			tempstr += " Level: ";
			tempstr += diagnostic_helper.get_DiagLevelString(subsystem_diagnostics.at(i).diagnostics.at(j).Level);
			tempstr += "\n";
		}
	}
	return tempstr;
}
eros::subsystem_diagnostic DiagnosticNodeProcess::get_eros_subsystem_diagnostic()
{
	for(std::size_t i = 0; i < eros_subsystem_diagnostic.Diagnostic_Type.size(); ++i)
	{
		eros_subsystem_diagnostic.Level.at(i) = subsystem_diagnostics.at(i).Level;
	}
	return eros_subsystem_diagnostic;

}