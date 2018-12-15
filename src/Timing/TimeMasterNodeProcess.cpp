#include "TimeMasterNodeProcess.h"
icarus_rover_v2::diagnostic  TimeMasterNodeProcess::finish_initialization()
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    	pps1_delay = 0;
    	time_since_last_1pps = 0.0;
    return diagnostic;
}
icarus_rover_v2::diagnostic TimeMasterNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		ready = true;

	}
	time_since_last_1pps += t_dt;
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
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
icarus_rover_v2::diagnostic TimeMasterNodeProcess::new_devicemsg(const icarus_rover_v2::device::ConstPtr& device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	return diag;
}
std::vector<icarus_rover_v2::diagnostic> TimeMasterNodeProcess::new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
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
std::vector<icarus_rover_v2::diagnostic> TimeMasterNodeProcess::check_programvariables()
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
bool TimeMasterNodeProcess::set_ppssource(std::string v)
{
	if(v == "self")
	{
		pps_source = v;
		return true;
	}
	return false;
}
bool TimeMasterNodeProcess::publish_1pps()
{
	if(time_since_last_1pps >= 1.0)
	{
		time_since_last_1pps = 0.0;
		return true;
	}
	return false;
}
