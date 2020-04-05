#include "TimeSlaveNodeProcess.h"
eros::diagnostic  TimeSlaveNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	reset();
	
	ros_master_uri = "";
	primary_time_server = "";
	unittesting_enabled = false;
	timesyncinfo.devicename = host_name;
	return diag;
}
eros::diagnostic TimeSlaveNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(ntp_initialized == false)
	{
		if(unittesting_enabled == true)
		{
			ntp_initialized = true;
		}
		else
		{
			if(const char* env_p = std::getenv("ROS_MASTER_URI"))
			{
				ros_master_uri = std::string(env_p);
				std::string tempstr  = ros_master_uri.substr(7);
				std::string tmp_server = tempstr.substr(0,tempstr.find(":"));
				if(tmp_server == "localhost")  //For when timeslave node is running on same device as timemaster
				{
					primary_time_server = "0.ubuntu.pool.n";
				}
				else
				{
					primary_time_server = tmp_server;
				}
				TimeServer server;
				server.name = primary_time_server;
				time_servers.push_back(server);
				timesyncinfo.servers.push_back(server.name);
				init_timeservers();
				ntp_initialized = true;
			}
		}
		

	}
	if(task_state == TASKSTATE_PAUSE)
	{

	}
	else if(task_state == TASKSTATE_INITIALIZING)
	{
		update_diagnostic(TIMING,NOTICE,INITIALIZING,"Initializing Time Servers.");
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
	else if(task_state == TASKSTATE_INITIALIZED)
	{
		request_statechange(TASKSTATE_RUNNING);
	}
	else if(task_state == TASKSTATE_RUNNING)
	{
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
	diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");
	}
	if(task_state == TASKSTATE_RUNNING)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
	}
	return diag;
}
eros::diagnostic TimeSlaveNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> TimeSlaveNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> TimeSlaveNodeProcess::check_programvariables()
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
void TimeSlaveNodeProcess::init_timeservers()
{
	for(std::size_t i = 0; i < time_servers.size(); ++i)
	{
		time_servers.at(i).level = ERROR;
		time_servers.at(i).delay = 0.0;
		time_servers.at(i).jitter = 0.0;
		time_servers.at(i).offset = 0.0;
		time_servers.at(i).updated_time = 0.0;
		time_servers.at(i).update_count = 0;
		timesyncinfo.delay.push_back(0.0);
		timesyncinfo.jitter.push_back(0.0);
		timesyncinfo.offset.push_back(0.0);
		timesyncinfo.time_updated.push_back(0.0);
		timesyncinfo.update_count.push_back(0.0);
	}

}
TimeSlaveNodeProcess::TimeServer TimeSlaveNodeProcess::get_timeserver(std::string name)
{
	for(std::size_t i = 0; i < time_servers.size(); ++i)
	{
		if(time_servers.at(i).name == name)
		{
			return time_servers.at(i);
		}
	}
	TimeSlaveNodeProcess::TimeServer empty;
	empty.name = "";
	empty.update_count = 0;
	return empty;
}
void TimeSlaveNodeProcess::set_timeserver(TimeSlaveNodeProcess::TimeServer server)
{
	for(std::size_t i = 0; i < time_servers.size(); ++i)
	{
		if(time_servers.at(i).name == server.name)
		{
			time_servers.at(i) = server;
			timesyncinfo.delay.at(i) = server.delay;
			timesyncinfo.jitter.at(i) = server.jitter;
			timesyncinfo.offset.at(i) = server.offset;
			timesyncinfo.time_updated.at(i) = server.updated_time;
			timesyncinfo.update_count.at(i) = server.update_count;
		}
	}
}
eros::diagnostic TimeSlaveNodeProcess::update_timeservers()
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < time_servers.size(); ++i)
	{
		diag = update_timeserver(time_servers.at(i).name);
		if(diag.Level > NOTICE)
		{
			return diag;
		}
	}

	
	for(std::size_t i = 0; i < time_servers.size(); ++i)
	{
		if(fabs(time_servers.at(i).delay) > 50.0)
		{
			time_servers.at(i).level = FATAL;
		}
		else if(fabs(time_servers.at(i).delay) > 20.0)
		{
			time_servers.at(i).level = ERROR;
		}
		else if(fabs(time_servers.at(i).delay) > 10.0)
		{
			time_servers.at(i).level = WARN;
		}
		else
		{
			time_servers.at(i).level = NOTICE;
		}
		if(time_servers.at(i).update_count == 0)
		{
			time_servers.at(i).level = WARN;
		}
	}
	if(time_servers.size() == 0)
	{
		char tempstr[512];
		sprintf(tempstr,"No Time Servers Defined.");
		diag = update_diagnostic(TIMING,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
	}
	else
	{
		char tempstr[512];
		switch(time_servers.at(0).level)
		{
		case INFO:
			sprintf(tempstr,"Primary Time Server OK.");
			diag = update_diagnostic(TIMING,INFO,DIAGNOSTIC_PASSED,std::string(tempstr));
			break;
		case NOTICE:
			sprintf(tempstr,"Primary Time Server OK.");
			diag = update_diagnostic(TIMING,NOTICE,DIAGNOSTIC_PASSED,std::string(tempstr));
			break;
		case WARN:
			sprintf(tempstr,"Primary Time Server Has bad Values.");
			diag = update_diagnostic(TIMING,WARN,DIAGNOSTIC_FAILED,std::string(tempstr));
			break;
		case ERROR:
			sprintf(tempstr,"Primary Time Server Has bad Values.");
			diag = update_diagnostic(TIMING,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
			break;
		case FATAL:
			sprintf(tempstr,"Primary Time Server Has bad Values.");
			diag = update_diagnostic(TIMING,FATAL,DIAGNOSTIC_FAILED,std::string(tempstr));
			break;
		default:
			sprintf(tempstr,"Primary Time Server Has unknown Values.");
			diag = update_diagnostic(TIMING,FATAL,DIAGNOSTIC_FAILED,std::string(tempstr));
			break;
		}
	}
	if(diag.Level <= NOTICE)
	{
		ready_to_arm = true;
	}
	return diag;
}
eros::diagnostic TimeSlaveNodeProcess::update_timeserver(std::string name)
{
	eros::diagnostic diag = root_diagnostic;
	TimeSlaveNodeProcess::TimeServer server = get_timeserver(name);
	char cmd[512];
	if(unittesting_enabled == false)
	{
		char unittesting_enabled[512];
		sprintf(cmd,"ntpq -p | grep %s",name.c_str());
		exec_result = exec(cmd,true);
	}
	std::vector<std::string> items;
	boost::split(items,exec_result,boost::is_any_of("\t "),boost::token_compress_on);
	int delay_index = -1;
	int offset_index = -1;
	int jitter_index = -1;
	if(items.size() == 10)
	{
		delay_index = 7;
		offset_index = 8;
		jitter_index = 9;
	}
	else if(items.size() == 11)
	{
		delay_index = 8;
		offset_index = 9;
		jitter_index = 10;
	}
	else
	{
		char tempstr[1024];
		sprintf(tempstr,"Improperly formatted NTP Command:\n%s\n Result: %s\n",cmd,exec_result.c_str());
		diag = update_diagnostic(TIMING,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
		return diag;

	}
	std::size_t found1 = items.at(0).find(name);
	std::size_t found2 = items.at(1).find(name);
	if((found1== std::string::npos) and (found2== std::string::npos))
	{
		char tempstr[512];
		sprintf(tempstr,"Time Server: %s Not Found",name.c_str());
		diag = update_diagnostic(TIMING,ERROR,DIAGNOSTIC_FAILED,std::string(tempstr));
		return diag;
	}
	server.delay = std::atof(items.at(delay_index).c_str())*1000.0;
	server.offset = std::atof(items.at(offset_index).c_str())*1000.0;
	server.jitter = std::atof(items.at(jitter_index).c_str())*1000.0;
	server.update_count++;
	server.updated_time = ros_time;
	set_timeserver(server);

	int host_index = -1;
	if(found1 != std::string::npos)
	{
		host_index = 0;
	}
	else
	{
		host_index = 1;
	}
	std::string host_field = items.at(host_index);
	char tempstr2[512];
	switch(host_field.at(0))
	{
	case '-':
		sprintf(tempstr2,"Time Server: %s Not Being Used by NTP.",name.c_str());
		diag = update_diagnostic(TIMING,WARN,DIAGNOSTIC_FAILED,std::string(tempstr2));
		return diag;
		break;
	case '+':
		sprintf(tempstr2,"Time Server: %s Being Used by NTP.",name.c_str());
		diag = update_diagnostic(TIMING,INFO,DIAGNOSTIC_PASSED,std::string(tempstr2));
		return diag;
		break;
	case '*':
		sprintf(tempstr2,"Time Server: %s Being Used by NTP.",name.c_str());
		diag = update_diagnostic(TIMING,INFO,DIAGNOSTIC_PASSED,std::string(tempstr2));
		return diag;
		break;
	default:
		sprintf(tempstr2,"Time Server: %s Not Being Used by NTP.",name.c_str());
		diag = update_diagnostic(TIMING,WARN,DIAGNOSTIC_FAILED,std::string(tempstr2));
		return diag;
		break;
	}


	return diag;
}
