#include "safety_node_process.h"
SafetyNodeProcess::SafetyNodeProcess()
{
    initialized = false;
    hostname = "";
    estop.name = "";
    estop.state = ESTOP_UNDEFINED;
    ready_to_arm = false;
}
SafetyNodeProcess::~SafetyNodeProcess()
{

}
void SafetyNodeProcess::init(std::string name,icarus_rover_v2::diagnostic diag)
{
    diagnostic = diag;
    estop.name = name;
    hostname = name;
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
icarus_rover_v2::diagnostic SafetyNodeProcess::update()
{
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
    
    diag.Level = INFO;
    diag.Diagnostic_Type = SOFTWARE;
    diag.Diagnostic_Message = NOERROR;
    diag.Description = "Updated";
    return diag;
}
icarus_rover_v2::diagnostic SafetyNodeProcess::new_devicemsg(icarus_rover_v2::device msg)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    if(initialized == false)
    {
        if(hostname == msg.DeviceName)
        {
            mydevice = msg;
            initialized = true;
        }
    }
    diag.Level = INFO;
    diag.Diagnostic_Type = COMMUNICATIONS;
    diag.Diagnostic_Message = NOERROR;
    char tempstr[512];
    sprintf(tempstr,"Initialized: %d",initialized);
    diag.Description = std::string(tempstr);
    return diag;
}
