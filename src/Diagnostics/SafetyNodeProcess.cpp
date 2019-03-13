#include "SafetyNodeProcess.h"
eros::diagnostic  SafetyNodeProcess::finish_initialization()
{
    eros::diagnostic diag = diagnostic;
    arm_switch = false;
    return diagnostic;
}
eros::diagnostic SafetyNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
    if(ready == true)
    {
        if((arm_switch == true))
		{
			ready_to_arm = true;
		}
		else
		{
			ready_to_arm = false;
		}
    }
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
eros::diagnostic SafetyNodeProcess::new_devicemsg(const eros::device::ConstPtr& t_device)
{
	eros::diagnostic diag = diagnostic;
    if(initialized == false)
	{
	}
	else
	{
		if(ready == false)
		{
			if(t_device->DeviceParent == host_name)
			{
				if(hat_present(t_device) == true)
				{
					diag.Level = WARN;
					diag.Diagnostic_Type = SOFTWARE;
					diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[512];
					sprintf(tempstr,"Hat: %s already loaded.",
							t_device->DeviceName.c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				std::size_t hat_message = t_device->DeviceType.find("Hat");
				if(hat_message != std::string::npos)
				{
					if(t_device->DeviceType == "TerminalHat")
					{
						bool arm_switch_found = false;
						for(std::size_t i = 0; i < t_device->pins.size(); i++)
						{
							if((t_device->pins.at(i).Function == "DigitalInput-Safety") and
									(t_device->pins.at(i).Name == "ArmSwitch"))
							{
								arm_switch_found = true;
							}

						}
						if(arm_switch_found == false)
						{
							diag.Level = ERROR;
							diag.Diagnostic_Type = SOFTWARE;
							diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
							char tempstr[512];
							sprintf(tempstr,"ArmSwitch Not Defined.");
							diag.Description = std::string(tempstr);
							return diag;
						}
					}
					else
					{
						diag.Level = NOTICE;
						diag.Diagnostic_Type = SOFTWARE;
						diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						char tempstr[512];
						sprintf(tempstr,"Hat Type: %s Not supported.",t_device->DeviceType.c_str());
						diag.Description = std::string(tempstr);
						return diag;
					}
                    eros::device device = convert_fromptr(t_device);
					hats.push_back(device);
					hats_running.push_back(false);
				}
			}
		}
		else
		{
		}
	}
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
	diag.Description = std::string(tempstr);
	return diag;
}
std::vector<eros::diagnostic> SafetyNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
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
std::vector<eros::diagnostic> SafetyNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = diagnostic;
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
eros::diagnostic SafetyNodeProcess::set_terminalhat_initialized()
{
	eros::diagnostic diag = diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			found = true;
			hats_running.at(i) = true;
			break; //Only 1 Terminal Hat supported.
		}
	}
	if(found == false)
	{
		diag.Level = WARN;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"TerminalHat Not Found");
		diag.Description = std::string(tempstr);
	}
	else
	{
		if(initialized == true)
		{
			ready = true;
		}
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"TerminalHat is now Initialized");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
int SafetyNodeProcess::get_pinnumber(std::string name)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if((hats.at(i).pins.at(j).Name == name))
				{
					return hats.at(i).pins.at(j).Number;
				}
			}
		}
	}
	return -1;
}
bool SafetyNodeProcess::set_pinvalue(std::string name,int v)
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if((hats.at(i).pins.at(j).Name == name))
				{
					hats.at(i).pins.at(j).Value = v;
					if(name == "ArmSwitch")
					{
						if(v == 1)
						{
							arm_switch = true;
						}
						else
						{

							arm_switch = false;
							diag.Diagnostic_Type = REMOTE_CONTROL;
							diag.Level = WARN;
							diag.Diagnostic_Message = ROVER_DISARMED;
							diagnostic = diag;
						}
					}
					return true;
				}
			}
		}
	}
	return false;
}
std::vector<eros::pin> SafetyNodeProcess::get_terminalhatpins(std::string Function)
{
	std::vector<eros::pin> pins;
	pins.clear();
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				eros::pin pin;
				if((hats.at(i).pins.at(j).Function == Function) or (Function == ""))
				{
					pin = hats.at(i).pins.at(j);
					pins.push_back(pin);
				}
			}
		}
	}
	return pins;
}
eros::diagnostic SafetyNodeProcess::new_armswitchmsg(std_msgs::Bool v)
{
	eros::diagnostic diag = diagnostic;
	arm_switch = v.data;

	diag.Diagnostic_Type = NOERROR;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	diag.Description = "ArmSwitch Updated";
	return diag;

}
bool SafetyNodeProcess::hat_present(const eros::device::ConstPtr& t_device)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceName == t_device->DeviceName))
		{
			return true;
		}
	}
	return false;
}
bool SafetyNodeProcess::is_hat_running(std::string devicetype,uint16_t id)
{
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
			found = true;
			if(hats_running.at(i) == true)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	return false;
}
eros::diagnostic SafetyNodeProcess::set_hat_running(std::string devicetype,uint16_t id)
{
	eros::diagnostic diag = diagnostic;
	eros::device hat;
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
			found = true;
			hats_running.at(i) = true;
			hat = hats.at(i);
		}
	}
	if(found == false)
	{
		diag.Level = WARN;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"Hat ID: %d Not Found.  Defined Hats: ",id);
		char tempstr2[512];
		for(int j = 0; j < hats.size(); j++)
		{
			sprintf(tempstr2,"%s:%ld,",hats.at(j).DeviceName.c_str(),hats.at(j).ID);
		}
		sprintf(tempstr,"%s%s\n",tempstr,tempstr2);
		diag.Description = std::string(tempstr);
	}
	else
	{
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"%s is now Initialized",hat.DeviceName.c_str());
		diag.Description = std::string(tempstr);
	}
	return diag;
}
