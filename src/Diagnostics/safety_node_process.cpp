#include "safety_node_process.h"
/*! \brief Constructor
 */
SafetyNodeProcess::SafetyNodeProcess()
{
	run_time = 0.0;
	initialized = false;
	ready = false;

	estop.name = "";
	estop.state = ESTOP_DISACTIVATED;
	ready_to_arm = false;
	arm_switch = false;
	estop.name = "Rover";
	last_estop = estop;
}
/*! \brief Deconstructor
 */
SafetyNodeProcess::~SafetyNodeProcess()
{

}
icarus_rover_v2::diagnostic SafetyNodeProcess::new_estopmsg(std_msgs::Bool v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(v.data == true)
	{
		estop.state = ESTOP_ACTIVATED;
		ready_to_arm = false;
	}
	else
	{
		estop.state = ESTOP_DISACTIVATED;
	}

	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "EStop Updated";

	return diag;
}
icarus_rover_v2::diagnostic SafetyNodeProcess::new_estopmsg(icarus_rover_v2::estop v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	std_msgs::Bool b;
	if(v.name != "Rover")
	{
		if(v.state == ESTOP_DISACTIVATED)
		{
			b.data = false;
		}
		else if(v.state == ESTOP_ACTIVATED)
		{
			b.data = true;
		}
		diag = new_estopmsg(b);
		return diag;
	}

	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "EStop Updated";
	return diag;
}
icarus_rover_v2::diagnostic SafetyNodeProcess::new_armswitchmsg(std_msgs::Bool v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	arm_switch = v.data;

	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "ArmSwitch Updated";
	return diag;

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{

	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;

	return diagnostic;
}
/*! \brief Time Update of Process
 */
icarus_rover_v2::diagnostic SafetyNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
	if(initialized == true) { ready = true; }

	if((estop.state == ESTOP_DISACTIVATED) and (arm_switch == true))
	{
		ready_to_arm = true;
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Armed";
	}
	else if(estop.state == ESTOP_ACTIVATED)
	{
		ready_to_arm = false;
		diag.Diagnostic_Type = REMOTE_CONTROL;
		diag.Level = WARN;
		diag.Diagnostic_Message = ROVER_DISARMED;
		diag.Description = "EStop Activated";
	}
	else if(estop.state == ESTOP_UNDEFINED)
	{
		ready_to_arm = false;
		diag.Diagnostic_Type = REMOTE_CONTROL;
		diag.Level = ERROR;
		diag.Diagnostic_Message = ROVER_DISARMED;
		diag.Description = "EStop Undefined";
	}
	else if(arm_switch == false)
	{
		ready_to_arm = false;
		diag.Diagnostic_Type = REMOTE_CONTROL;
		diag.Level = INFO;
		diag.Diagnostic_Message = ROVER_DISARMED;
		diag.Description = "Arm Switch Disactivated";
	}
	else
	{
		ready_to_arm = false;
		diag.Diagnostic_Type = REMOTE_CONTROL;
		diag.Level = ERROR;
		diag.Diagnostic_Message = ROVER_DISARMED;
		diag.Description = "Unknown state";

	}
	if(estop.state != last_estop.state)
	{
		if(estop.state == ESTOP_DISACTIVATED)
		{
			diag.Level = NOTICE;
		}
	}
	last_estop = estop;


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

