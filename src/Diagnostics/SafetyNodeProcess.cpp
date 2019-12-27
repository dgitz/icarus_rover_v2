#include "SafetyNodeProcess.h"
eros::diagnostic  SafetyNodeProcess::finish_initialization()
{
    eros::diagnostic diag = root_diagnostic;
    arm_switch = false;
    return diag;
}
eros::diagnostic SafetyNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
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
	if((is_initialized() == true) and (is_ready() == true))
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
	}
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");

	}

	return diag;
}
eros::diagnostic SafetyNodeProcess::new_devicemsg(const eros::device::ConstPtr& t_device)
{
	eros::diagnostic diag = root_diagnostic;
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
					char tempstr[512];
					sprintf(tempstr,"Hat: %s already loaded.",
							t_device->DeviceName.c_str());
					diag = update_diagnostic(DATA_STORAGE,WARN,DEVICE_NOT_AVAILABLE,std::string(tempstr));
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
									(t_device->pins.at(i).ConnectedDevice == "ArmSwitch"))
							{
								arm_switch_found = true;
							}

						}
						if(arm_switch_found == false)
						{
							char tempstr[512];
							sprintf(tempstr,"ArmSwitch Not Defined.");
							diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
							return diag;
						}
					}
					else
					{
						char tempstr[512];
						sprintf(tempstr,"Hat Type: %s Not supported.",t_device->DeviceType.c_str());
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
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
	char tempstr[512];
	sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
	diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,std::string(tempstr));
	return diag;
}
std::vector<eros::diagnostic> SafetyNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
std::vector<eros::diagnostic> SafetyNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
eros::diagnostic SafetyNodeProcess::set_terminalhat_initialized()
{
	eros::diagnostic diag = root_diagnostic;
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
		char tempstr[512];
		sprintf(tempstr,"TerminalHat Not Found");
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	}
	else
	{
		if(initialized == true)
		{
			ready = true;
		}
		char tempstr[512];
		sprintf(tempstr,"TerminalHat is now Initialized");
		diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,std::string(tempstr));
	}
	return diag;
}
bool SafetyNodeProcess::set_pinvalue(std::string name,int v)
{
	eros::diagnostic diag = root_diagnostic;
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
							diag = update_diagnostic(REMOTE_CONTROL,NOTICE,ROVER_DISARMED,"Rover Disarmed");
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
	eros::diagnostic diag = root_diagnostic;
	arm_switch = v.data;
	diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"ArmSwitch Updated.");
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
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
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
	eros::diagnostic diag = root_diagnostic;
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
		char tempstr[512];
		sprintf(tempstr,"Hat ID: %d Not Found.  Defined Hats: ",id);
		char tempstr2[512];
		for(std::size_t j = 0; j < hats.size(); j++)
		{
			sprintf(tempstr2,"%s:%ld,",hats.at(j).DeviceName.c_str(),hats.at(j).ID);
		}
		sprintf(tempstr,"%s%s\n",tempstr,tempstr2);
		diag = update_diagnostic(DATA_STORAGE,ERROR,DEVICE_NOT_AVAILABLE,std::string(tempstr));
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"%s is now Initialized",hat.DeviceName.c_str());
		diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,std::string(tempstr));
	}
	return diag;
}
