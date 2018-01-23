#include "safety_node_process.h"
/*! \brief Constructor
 */
SafetyNodeProcess::SafetyNodeProcess()
{
	run_time = 0.0;
	initialized = false;
    ready = false;

    estop.name = "";
    estop.state = ESTOP_UNDEFINED;
    ready_to_arm = false;
}
/*! \brief Deconstructor
 */
SafetyNodeProcess::~SafetyNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;

	estop.name = hostname;
	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::update(double dt)
{
	run_time += dt;
   	if(initialized == true) { ready = true; }
    icarus_rover_v2::diagnostic diag = diagnostic;
    if(estop.state == ESTOP_DISACTIVATED)
    {
        ready_to_arm = true;
    }
    else if(estop.state == ESTOP_UNDEFINED)
    {
        ready_to_arm = false;
    }
    else if(estop.state == ESTOP_ACTIVATED)
    {
        ready_to_arm = false;
    }
    
    diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "Node Running";
	diagnostic = diag;
    return diag;
}
/*! \brief Setup Process Device info
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::new_devicemsg(icarus_rover_v2::device device)
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
std::vector<icarus_rover_v2::diagnostic> SafetyNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
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
std::vector<icarus_rover_v2::diagnostic> SafetyNodeProcess::check_program_variables()
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
icarus_rover_v2::diagnostic SafetyNodeProcess::new_pinvalue(int v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(v == 0)
	{
		estop.state = ESTOP_DISACTIVATED;
	}
	else if(v == 1)
	{
		estop.state = ESTOP_ACTIVATED;
	}
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Updated Pin: %d to value: %d EStop: %d",PIN_ESTOP,v,estop.state);
	diag.Description = std::string(tempstr);
	return diag;
}
icarus_rover_v2::diagnostic SafetyNodeProcess::new_pinmsg(icarus_rover_v2::pin msg)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    if((msg.Number == PIN_ESTOP) and (mydevice.DeviceName == msg.ParentDevice))
    {
        if(msg.Value == 0)
        {
            estop.state = ESTOP_DISACTIVATED;
        }
        else if(msg.Value == 1)
        {
            estop.state = ESTOP_ACTIVATED;
        }
    }
    diag.Level = INFO;
    diag.Diagnostic_Type = COMMUNICATIONS;
    diag.Diagnostic_Message = NOERROR;
    char tempstr[512];
    sprintf(tempstr,"Updated Pin %s:%d to value: %d.",msg.ParentDevice.c_str(),msg.Number,msg.Value);
    diag.Description = std::string(tempstr);
    return diag;
}

