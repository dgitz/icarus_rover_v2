#include "TimeMasterNodeProcess.h"
eros::diagnostic TimeMasterNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	reset();
	if (pps_source == "self")
	{
		update_diagnostic(TIMING, INFO, NOERROR, "Using Self Timebase.");
		update_diagnostic(SENSORS, INFO, NOERROR, "Using Self Timebase.");
	}

	return diag;
}
eros::diagnostic TimeMasterNodeProcess::update(double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(task_state == TASKSTATE_PAUSE)
	{

	}
	else if(task_state == TASKSTATE_INITIALIZED)
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
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
	else if (get_taskstate() == TASKSTATE_RUNNING)
	{
		update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
	}
	time_since_last_1pps += t_dt;
	diag = update_baseprocess(t_dt, t_ros_time);
	if (diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running.");
	}
	return diag;
}
eros::diagnostic TimeMasterNodeProcess::new_devicemsg(__attribute__((unused)) const eros::device::ConstPtr &device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> TimeMasterNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
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
std::vector<eros::diagnostic> TimeMasterNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true)
	{
		diag = update_diagnostic(SOFTWARE, INFO, DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, WARN, DIAGNOSTIC_FAILED, "Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
bool TimeMasterNodeProcess::set_ppssource(std::string v)
{
	if (v == "self")
	{
		pps_source = v;
		return true;
	}
	return false;
}
bool TimeMasterNodeProcess::publish_1pps()
{
	if (time_since_last_1pps >= 1.0)
	{
		time_since_last_1pps = 0.0;
		return true;
	}
	return false;
}
