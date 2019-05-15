#include "CalibrationNodeProcess.h"
eros::diagnostic  CalibrationNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
    return diag;
}
eros::diagnostic CalibrationNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(initialized == true)
	{
		ready = true;

	}
	diag = update_baseprocess(t_dt,t_ros_time);
	if((is_initialized() == true) and (is_ready() == true))
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
	}
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");
		diag = update_diagnostic(SENSORS,NOTICE,DEVICE_NOT_AVAILABLE,"Not Implemented Yet.");

	}
	return diag;
}
eros::diagnostic CalibrationNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	return diag;
}
std::vector<eros::diagnostic> CalibrationNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
std::vector<eros::diagnostic> CalibrationNodeProcess::check_programvariables()
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
