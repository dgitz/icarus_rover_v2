#include "webserver_node_process.h"
/*! \brief Constructor
 */
WebServerNodeProcess::WebServerNodeProcess()
{
	run_time = 0.0;
	initialized = false;
	ready = false;
}
/*! \brief Deconstructor
 */
WebServerNodeProcess::~WebServerNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic WebServerNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic WebServerNodeProcess::update(double dt)
{
	run_time += dt;
	if(initialized == true) { ready = true; }
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
icarus_rover_v2::diagnostic WebServerNodeProcess::new_devicemsg(icarus_rover_v2::device device)
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
std::vector<icarus_rover_v2::diagnostic> WebServerNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
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
std::vector<icarus_rover_v2::diagnostic> WebServerNodeProcess::check_program_variables()
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
icarus_rover_v2::diagnostic WebServerNodeProcess::update_systemdevicelist(std::vector<icarus_rover_v2::device> list)
{
	icarus_rover_v2::diagnostic diag=diagnostic;
	for(std::size_t i = 0; i < list.size(); i++)
	{
		bool found = false;
		for(std::size_t j = 0; j < system_devicelist.size(); j++)
		{
			if(system_devicelist.at(j).DeviceName == list.at(i).DeviceName)
			{
				system_devicelist.at(j) = list.at(i);
				found = true;
			}

		}
		if(found == false)
		{
			printf("Adding: %s\n",list.at(i).DeviceName.c_str());
			system_devicelist.push_back(list.at(i));
		}

	}
	diag.Diagnostic_Type = SOFTWARE;
			diag.Level = INFO;
			diag.Diagnostic_Message = INITIALIZING;
			char tempstr[512];
			sprintf(tempstr,"Received info for System Device.");
			diag.Description = std::string(tempstr);
	return diag;
}
void WebServerNodeProcess::new_armedstatus(uint8_t v)
{
	armed_status = v;
}
