#include "diagnostic_node_process.h"
/*! \brief Constructor
 */
DiagnosticNodeProcess::DiagnosticNodeProcess()
{
	ready_to_arm = false;
	run_time = 0.0;
	initialized = false;
	node_name = "";
	last_1pps_timer = 0.0;
	last_01pps_timer = 0.0;
	last_cmddiagnostic_timer = 0.0;
}
/*! \brief Deconstructor
 */
DiagnosticNodeProcess::~DiagnosticNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic DiagnosticNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
   	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic DiagnosticNodeProcess::update(double dt)
{
	run_time += dt;
	last_1pps_timer += dt;
	last_01pps_timer += dt;
	last_cmddiagnostic_timer += dt;
    if(initialized == true) { ready = true; }
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool status = true;
	if(last_1pps_timer > 5.0)
	{
		status = false;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = MISSING_HEARTBEATS;
		char tempstr[512];
		sprintf(tempstr,"Have not received 1PPS in: %f Seconds",last_1pps_timer);
		diag.Description = std::string(tempstr);
	}
	if(last_01pps_timer > 30.0)
	{
		status = false;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = MISSING_HEARTBEATS;
		char tempstr[512];
		sprintf(tempstr,"Have not received 01PPS in: %f Seconds",last_01pps_timer);
		diag.Description = std::string(tempstr);
	}
	if(last_cmddiagnostic_timer > 2.0)
	{
		status = false;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
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
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic DiagnosticNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == myhostname)
	{

    }
    return diag;
}
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> DiagnosticNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command ==  ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		last_cmddiagnostic_timer = 0.0;
		if(cmd.Option1 == LEVEL1)
		{
		}
		else if(cmd.Option1 == LEVEL2)
		{
			diaglist = check_program_variables();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL3)
		{
			diaglist = check_tasks();
			return diaglist;
		}
		else if(cmd.Option1 == LEVEL4)
		{
		}
	}
	diaglist.push_back(diag);
	return diaglist;
}
/*! \brief Self-Diagnostic-Check Program Variables
 */
std::vector<icarus_rover_v2::diagnostic> DiagnosticNodeProcess::check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;

	if(status == true)
	{
		icarus_rover_v2::diagnostic diag=diagnostic;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		icarus_rover_v2::diagnostic diag=diagnostic;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
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
void DiagnosticNodeProcess::new_resourcemsg(std::string topicname,icarus_rover_v2::resource resource)
{
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).resource_topic == topicname)
		{
			TaskList.at(i).last_resource_received = run_time;
			TaskList.at(i).CPU_Perc = resource.CPU_Perc;
			TaskList.at(i).RAM_MB = resource.RAM_MB;
			TaskList.at(i).PID = resource.PID;
		}
	}

	std::size_t resource_available_topic = topicname.find("resource_available");
	if(resource_available_topic != std::string::npos)
	{
		bool found = true;
		for(std::size_t i = 0; i < DeviceResourceAvailableList.size();i++)
		{
			if(DeviceResourceAvailableList.at(i).Device_Name == resource.Node_Name)
			{
				found = false;
				DeviceResourceAvailableList.at(i).CPU_Perc_Available = resource.CPU_Perc;
				DeviceResourceAvailableList.at(i).RAM_Mb_Available = resource.RAM_MB;
				break;
			}
		}
		if(found == true)
		{
			DeviceResourceAvailable newdevice;
			newdevice.Device_Name = resource.Node_Name;
			newdevice.CPU_Perc_Available = resource.CPU_Perc;
			newdevice.RAM_Mb_Available = resource.RAM_MB;
			DeviceResourceAvailableList.push_back(newdevice);
		}
	}
}
void DiagnosticNodeProcess::new_diagnosticmsg(std::string topicname,icarus_rover_v2::diagnostic diagnostic)
{
	bool add_me = true;
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).diagnostic_topic == topicname)
		{
			TaskList.at(i).last_diagnostic_received = run_time;
			TaskList.at(i).last_diagnostic_level = diagnostic.Level;
		}
	}
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
		/*
		if(newTask.PID <= 0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s does not have a valid PID.",newTask.Task_Name.c_str());
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = HIGH_RESOURCE_USAGE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		double resource_time_duration = measure_time_diff(ros::Time::now(),newTask.last_resource_received);
		//printf("Task: %s Resource Time: %f\r\n",newTask.Task_Name.c_str(),resource_time_duration);
		if( resource_time_duration > 5.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported resources used in %.1f seconds",newTask.Task_Name.c_str(),resource_time_duration);
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}

		double diagnostic_time_duration = measure_time_diff(ros::Time::now(),newTask.last_diagnostic_received);
		if( resource_time_duration > 5.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported diagnostics in %.1f seconds",newTask.Task_Name.c_str(),diagnostic_time_duration);
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		*/
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
icarus_rover_v2::diagnostic DiagnosticNodeProcess::new_1ppsmsg()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	last_1pps_timer = 0.0;
	return diag;
}
icarus_rover_v2::diagnostic DiagnosticNodeProcess::new_01ppsmsg()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	last_01pps_timer = 0.0;
	return diag;
}
