#include "timemaster_node_process.h"
/*! \brief Constructor
 */
TimeMasterNodeProcess::TimeMasterNodeProcess()
{
	run_time = 0.0;
	initialized = false;
    pps01_delay = 0;
    pps1_delay = 0;
    time_since_last_1pps = 0.0;
    time_since_last_01pps = 0.0;
}
/*! \brief Deconstructor
 */
TimeMasterNodeProcess::~TimeMasterNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic TimeMasterNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
   	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic TimeMasterNodeProcess::update(double dt)
{
	run_time += dt;
	if((mydevice.BoardCount == 0) and (mydevice.SensorCount == 0))
    {
        if(initialized == true) { ready = true; }
    }
    time_since_last_1pps += dt;
    time_since_last_01pps += dt;
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Node Running";
	diagnostic = diag;
	return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic TimeMasterNodeProcess::new_devicemsg(icarus_rover_v2::device device)
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
std::vector<icarus_rover_v2::diagnostic> TimeMasterNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	if (cmd.Command ==  ROVERCOMMAND_RUNDIAGNOSTIC)
	{
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
std::vector<icarus_rover_v2::diagnostic> TimeMasterNodeProcess::check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag=diagnostic;
	bool status = true;

	if(status == true)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
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
bool TimeMasterNodeProcess::publish_01pps()
{
   if(time_since_last_01pps >= 10.0)
    {
        time_since_last_01pps = 0.0;
        return true;
    }
    return false; 
}