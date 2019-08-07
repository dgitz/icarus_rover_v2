#include "TimeMasterNodeProcess.h"
eros::diagnostic TimeMasterNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	pps1_delay = 0;
	time_since_last_1pps = 0.0;
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
	if (initialized == true)
	{
		ready = true;
	}
	if ((is_initialized() == true) and (is_ready() == true))
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
