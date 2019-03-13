#include "HatControllerNodeProcess.h"
eros::diagnostic  HatControllerNodeProcess::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	ready_to_arm = false;
	armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	hats.clear();
	hats_running.clear();
	time_sincelast_pps = 0.0;
	pps_counter = 0;
	analyze_timing = false;
	timing_diff.clear();
	return diagnostic;
}
eros::diagnostic HatControllerNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	if((mydevice.HatCount == 0) and (mydevice.SensorCount == 0))
	{
		if(initialized == true) { ready = true; }
	}
	bool hats_ready = true;
	for(std::size_t i = 0; i < hats_running.size(); i++)
	{
		if(hats_running.at(i) == true) { hats_ready = hats_ready and true; }
		else { hats_ready = false; }
	}
	if(hats_running.size() == 0) { hats_ready = false; }
	bool status = true;
	if((hats_ready == true))
	{
		status = status and true;
	}
	else
	{
		status = false;
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = INITIALIZING;
		diag.Description = "Initializing";
	}
	ready_to_arm = status;
	if(status == false)
	{
		armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;

	}
	else
	{
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running";
	}
	diagnostic = diag;
	return diag;
}
std::vector<eros::diagnostic> HatControllerNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
std::vector<eros::diagnostic> HatControllerNodeProcess::check_programvariables()
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
bool HatControllerNodeProcess::sensors_initialized()
{
	bool sensors_init = true;
	if((sensors.size() == 0) and (mydevice.SensorCount == 0))
	{
		return true;
	}
	for(std::size_t i=0; i < sensors.size(); i++)
	{
		if(sensors.at(i).initialized == false)
		{
			sensors_init = false;
		}
	}
	return sensors_init;
}
std::string HatControllerNodeProcess::map_PinFunction_ToString(int function)
{
	switch(function)
	{
	case PINMODE_UNDEFINED:						return "Undefined";					break;
	case PINMODE_DIGITAL_OUTPUT: 				return "DigitalOutput"; 			break;
	case PINMODE_DIGITAL_INPUT: 				return "DigitalInput"; 				break;
	case PINMODE_ANALOG_INPUT: 					return "AnalogInput"; 				break;
	case PINMODE_ULTRASONIC_INPUT: 				return "UltraSonicSensorInput"; 	break;
	case PINMODE_QUADRATUREENCODER_INPUT: 		return "QuadratureEncoderInput";	break;
	case PINMODE_PWM_OUTPUT: 					return "PWMOutput"; 				break;
	case PINMODE_PWM_OUTPUT_NON_ACTUATOR:		return "PWMOutput-NonActuator";		break;
	case PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR:	return "DigitalOutput-NonActuator"; break;
	default: 									return ""; 							break;
	}
}
int HatControllerNodeProcess::map_PinFunction_ToInt(std::string Function)
{
	if(Function == "DigitalInput")						{	return PINMODE_DIGITAL_INPUT;				}
	else if(Function == "DigitalOutput")				{	return PINMODE_DIGITAL_OUTPUT;				}
	else if(Function == "AnalogInput")					{	return PINMODE_ANALOG_INPUT;				}
	else if(Function == "UltraSonicSensorInput")		{	return PINMODE_ULTRASONIC_INPUT;			}
	else if(Function == "PWMOutput")					{	return PINMODE_PWM_OUTPUT;					}
	else if(Function == "PWMOutput-NonActuator") 		{ 	return PINMODE_PWM_OUTPUT_NON_ACTUATOR; 	}
	else if(Function == "DigitalOutput-NonActuator")	{ 	return PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR;	}
	else 												{ 	return PINMODE_UNDEFINED; 					}
}
eros::diagnostic HatControllerNodeProcess::new_devicemsg(const eros::device::ConstPtr& t_newdevice)
{
	eros::diagnostic diag = diagnostic;
	eros::device device = convert_fromptr(t_newdevice);
	if(initialized == false)
	{
	}
	else
	{
		if(ready == false)
		{
			if(t_newdevice->DeviceParent == host_name)
			{
				if(hat_present(t_newdevice) == true)
				{
					diag.Level = WARN;
					diag.Diagnostic_Type = SOFTWARE;
					diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[512];
					sprintf(tempstr,"Hat: %s already loaded.",
							t_newdevice->DeviceName.c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				std::size_t hat_message = t_newdevice->DeviceType.find("Hat");
				if(hat_message != std::string::npos)
				{
					if(t_newdevice->DeviceType == "ServoHat")
					{
						for(std::size_t i = 0; i < t_newdevice->pins.size(); i++)
						{
							if((t_newdevice->pins.at(i).Function == "PWMOutput") or
									(t_newdevice->pins.at(i).Function == "PWMOutput-NonActuator")) {
							}
							else
							{
								diag.Level = ERROR;
								diag.Diagnostic_Type = SOFTWARE;
								diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
								char tempstr[512];
								sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
										t_newdevice->DeviceType.c_str(),t_newdevice->pins.at(i).Function.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}
						}
					}
					else if(t_newdevice->DeviceType == "TerminalHat")
					{
						for(std::size_t i = 0; i < t_newdevice->pins.size(); i++)
						{
							if((t_newdevice->pins.at(i).Function == "DigitalInput") or
									(t_newdevice->pins.at(i).Function == "DigitalInput-Safety") or
									(t_newdevice->pins.at(i).Function == "DigitalOutput-NonActuator") or
									(t_newdevice->pins.at(i).Function == "DigitalOutput")) {}

							else
							{
								diag.Level = ERROR;
								diag.Diagnostic_Type = SOFTWARE;
								diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
								char tempstr[512];
								sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
										t_newdevice->DeviceType.c_str(),t_newdevice->pins.at(i).Function.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}
						}
					}
					else if(t_newdevice->DeviceType == "GPIOHat")
					{
						for(std::size_t i = 0; i < t_newdevice->pins.size(); i++)
						{
							if(t_newdevice->pins.at(i).Function == map_PinFunction_ToString(PINMODE_ULTRASONIC_INPUT))
							{
								Sensor new_sensor;
								new_sensor.initialized = false;
								new_sensor.signal.value = 0.0;
								new_sensor.name = t_newdevice->pins.at(i).ConnectedSensor;
								new_sensor.connected_hat = device;
								new_sensor.connected_pin = t_newdevice->pins.at(i);
								new_sensor.status = SIGNALSTATE_UNDEFINED;

								sensors.push_back(new_sensor);
								if(load_sensorinfo(new_sensor.name) == false)
								{
									diag.Level = ERROR;
									diag.Diagnostic_Type = SOFTWARE;
									diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
									char tempstr[512];
									sprintf(tempstr,"Unable to load info for Sensor: %s",new_sensor.name.c_str());
									diag.Description = std::string(tempstr);
									return diag;
								}

							}
							else
							{
								diag.Level = ERROR;
								diag.Diagnostic_Type = SOFTWARE;
								diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
								char tempstr[512];
								sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
										t_newdevice->DeviceType.c_str(),t_newdevice->pins.at(i).Function.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}
						}
					}
					else
					{
						diag.Level = ERROR;
						diag.Diagnostic_Type = SOFTWARE;
						diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						char tempstr[512];
						sprintf(tempstr,"Hat Type: %s Not supported.",t_newdevice->DeviceType.c_str());
						diag.Description = std::string(tempstr);
						return diag;
					}
					hats.push_back(device);
					hats_running.push_back(false);
					if(hats.size() == mydevice.HatCount)
					{
						ready = true;
					}
				}
			}
		}
		else
		{
		}
	}
	if((hats.size() == mydevice.HatCount) and
			(sensors_initialized() == true))
	{
		ready = true;
	}
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
	diag.Description = std::string(tempstr);
	return diag;
}

eros::diagnostic HatControllerNodeProcess::new_pinmsg(const eros::pin::ConstPtr& t_msg)
{
	eros::diagnostic diag = diagnostic;
	int found = -1;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).DeviceName == t_msg->ParentDevice)
		{
			found = 0;
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if(hats.at(i).pins.at(j).Number == t_msg->Number)
				{
					found = 1;
					eros::pin pin = convert_fromptr(t_msg);
					hats.at(i).pins.at(j) = pin;
					found = true;
					break;
				}
			}
		}
	}


	if(found == -1)
	{
		diag.Level = INFO;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"Pin Msg for Device: %s but not for me.",t_msg->ParentDevice.c_str());
		diag.Description = std::string(tempstr);
	}
	else if(found == 0)
	{
		diag.Level = WARN;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"Pin Msg for My Hat but Pin not found: %s:%d",t_msg->ParentDevice.c_str(),t_msg->Number);
		diag.Description = std::string(tempstr);
	}
	else
	{
		if(analyze_timing == true)
		{
			timing_diff.push_back(t_msg->stamp.toSec()-run_time);
			if(timing_diff.size() > TIMING_BUFFER_LENGTH)
			{
				timing_diff.erase(timing_diff.begin());
			}
		}

		diag.Level = INFO;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"Updated Pin %s:%d to value: %d.",t_msg->ParentDevice.c_str(),t_msg->Number,t_msg->Value);
		diag.Description = std::string(tempstr);

	}
	return diag;

}
eros::device HatControllerNodeProcess::find_hat(uint8_t hatid)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).ID == hatid)
		{
			return hats.at(i);
		}
	}
	eros::device empty_device;
	empty_device.DeviceName = "";
	empty_device.DeviceType = "";
	return empty_device;
}
eros::pin HatControllerNodeProcess::find_pin(const eros::device::ConstPtr& hat,std::string pinfunction,uint8_t pinnumber)
{
	for(std::size_t i = 0; i < hat->pins.size(); i++)
	{
		if((hat->pins.at(i).Function == pinfunction) and (hat->pins.at(i).Number == pinnumber))
		{
			return hat->pins.at(i);
		}
	}
	eros::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
eros::diagnostic HatControllerNodeProcess::new_message_GetDIOPort1(uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4)
{
	bool any_error = false;
	uint16_t v[6] = {v1,v2,v3,v4};
	eros::diagnostic diag = diagnostic;
	eros::device hat = find_hat(hatid);
	if(hat.DeviceName == "")
	{
		char tempstr[1024];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		sprintf(tempstr,"Hat ID: %d Not Found.  Defined Hats: ",hatid);
		char tempstr2[512];
		for(int j = 0; j < hats.size(); j++)
		{
			sprintf(tempstr2,"%s:%ld,",hats.at(j).DeviceName.c_str(),hats.at(j).ID);
		}
		sprintf(tempstr,"%s%s\n",tempstr,tempstr2);
		diag.Description = std::string(tempstr);
		return diag;
	}
	for(uint8_t i = 0; i < 4; i++)
	{
		eros::device::ConstPtr hat_ptr(new eros::device(hat));
		eros::pin pin = find_pin(hat_ptr,"UltraSonicSensorInput",i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if(pin.Name != "")
		{
			if(update_sensor(hat_ptr,pin_ptr,tov,(double)v[i]) == false)
			{
				any_error = true;
				char tempstr[255];
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = WARN;
				diag.Diagnostic_Message = DROPPING_PACKETS;
				sprintf(tempstr,"Unable to Update Pin: UltraSonicSensorInput:%d",i);
				diag.Description = std::string(tempstr);
			}
		}
		else
		{
			any_error = true;
			char tempstr[255];
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Level = WARN;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			sprintf(tempstr,"Unable to Find Pin: UltraSonicSensorInput:%d",i);
			diag.Description = std::string(tempstr);
		}
	}
	if(any_error == false)
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		sprintf(tempstr,"Updated");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
bool HatControllerNodeProcess::update_sensor(const eros::device::ConstPtr& t_device,eros::pin::ConstPtr& t_pin,double tov,double value)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if((sensors.at(i).connected_hat.DeviceName == t_device->DeviceName) and
				(sensors.at(i).connected_pin.Name == t_pin->Name))
		{
			sensors.at(i).signal.tov = convert_time(tov);
			sensors.at(i).signal.status = SIGNALSTATE_UPDATED;
			if(sensors.at(i).convert == false)
			{
				sensors.at(i).signal.value = value;
			}
			else
			{
				sensors.at(i).signal.value = map_input_to_output(value,sensors.at(i).min_inputvalue,
						sensors.at(i).max_inputvalue,
						sensors.at(i).min_inputvalue,
						sensors.at(i).max_outputvalue);
			}
			sensors.at(i).signal.rms = -1;
			return true;
		}
	}
	return false;
}
double HatControllerNodeProcess::get_timedelay()
{
	std::vector<double> temp = timing_diff;
	double t = 0.0;
	if(temp.size() <= (TIMING_BUFFER_LENGTH/2))
	{
		return 0.0;
	}
	for(std::size_t i = 0; i < temp.size(); i++)
	{
		t += temp.at(i);
	}
	return t/(double)(temp.size());
}
eros::diagnostic HatControllerNodeProcess::new_pinsmsg(const eros::iopins::ConstPtr& t_msg)
{
	eros::diagnostic diag = diagnostic;
	int found = -1;
	for(std::size_t k = 0; k < t_msg->pins.size(); k++)
	{
		for(std::size_t i = 0; i < hats.size(); i++)
		{
			if(hats.at(i).DeviceName == t_msg->pins.at(k).ParentDevice)
			{
				found = 0;
				for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
				{
					if(hats.at(i).pins.at(j).Number == t_msg->pins.at(k).Number)
					{
						found = 1;
						hats.at(i).pins.at(j) = t_msg->pins.at(k);
						found = true;
						break;
					}
				}
			}
		}
	}
	/*
    if(found == -1)
    {
        diag.Level = INFO;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Pin Msg for Device: %s but not for me.",msg.ParentDevice.c_str());
        diag.Description = std::string(tempstr);
    }
    else if(found == 0)
    {
        diag.Level = WARN;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"Pin Msg for My Hat but Pin not found: %s:%d",msg.ParentDevice.c_str(),msg.Number);
        diag.Description = std::string(tempstr);
    }
    else
    {
        diag.Level = INFO;
        diag.Diagnostic_Type = COMMUNICATIONS;
        diag.Diagnostic_Message = NOERROR;
        char tempstr[512];
        sprintf(tempstr,"Updated Pin %s:%d to value: %d.",msg.ParentDevice.c_str(),msg.Number,msg.Value);
        diag.Description = std::string(tempstr);
    }
	 */
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Updated Pins %d",(int)t_msg->pins.size());
	diag.Description = std::string(tempstr);
	return diag;

}


eros::diagnostic HatControllerNodeProcess::new_armedstatemsg(uint8_t msg)
{
	eros::diagnostic diag = diagnostic;
	armed_state = msg;
	diag.Level = INFO;
	diag.Diagnostic_Type = SOFTWARE;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Rover Armed State: %d Processed.",msg);
	diag.Description = std::string(tempstr);
	return diag;
}
bool HatControllerNodeProcess::is_hat_running(std::string devicetype,uint16_t id)
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
eros::diagnostic HatControllerNodeProcess::set_hat_running(std::string devicetype,uint16_t id)
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
eros::diagnostic HatControllerNodeProcess::set_terminalhat_initialized()
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
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"TerminalHat is now Initialized");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
std::vector<uint16_t> HatControllerNodeProcess::get_servohataddresses()
{
	std::vector<uint16_t> addresses;
	addresses.clear();
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).DeviceType == "ServoHat")
		{
			addresses.push_back(hats.at(i).ID);
		}
	}
	return addresses;
}
std::vector<uint16_t> HatControllerNodeProcess::get_gpiohataddresses()
{
	std::vector<uint16_t> addresses;
	addresses.clear();
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).DeviceType == "GPIOHat")
		{
			addresses.push_back(hats.at(i).ID);
		}
	}
	return addresses;
}
bool HatControllerNodeProcess::set_terminalhatpinvalue(std::string name,int v)
{
	bool found = false;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if(hats.at(i).pins.at(j).Name == name)
				{
					hats.at(i).pins.at(j).Value = v;
					found = true;
				}
			}
		}
	}
	return found;
}
std::vector<eros::pin> HatControllerNodeProcess::get_terminalhatpins(std::string Function,bool match_exact)
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
				if(((match_exact == true) and (hats.at(i).pins.at(j).Function == Function)) or
						((match_exact == false) and (hats.at(i).pins.at(j).Function.find(Function) != std::string::npos)))
				{
					pin = hats.at(i).pins.at(j);
					if((Function == "DigitalOutput") and (armed_state != ARMEDSTATUS_ARMED))
					{
						pin.Value = hats.at(i).pins.at(j).DefaultValue;
					}
					else
					{
						pin.Value = hats.at(i).pins.at(j).Value;
					}
					pins.push_back(pin);
				}
			}
		}
	}
	return pins;
}
std::vector<eros::pin> HatControllerNodeProcess::get_gpiohatpins(uint16_t id)
{
	std::vector<eros::pin> pins;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "GPIOHat") and (hats.at(i).ID == id))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				eros::pin pin;
				if(hats.at(i).pins.at(j).Function == "UltraSonicSensorInput")
				{
					pin = hats.at(i).pins.at(j);
					pins.push_back(pin);

				}
				else
				{
					pin = hats.at(i).pins.at(j);
					pin.Value = hats.at(i).pins.at(j).DefaultValue;
					pins.push_back(pin);
				}
			}
		}
	}
	return pins;
}
std::vector<eros::pin> HatControllerNodeProcess::get_servohatpins(uint16_t id)
{
	std::vector<eros::pin> pins;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "ServoHat") and (hats.at(i).ID == id))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				eros::pin pin;
				if(hats.at(i).pins.at(j).Function == "PWMOutput-NonActuator")
				{
					pin = hats.at(i).pins.at(j);
					pins.push_back(pin);

				}
				else if(armed_state == ARMEDSTATUS_ARMED)
				{
					if(hats.at(i).pins.at(j).Function == "PWMOutput")
					{
						pin = hats.at(i).pins.at(j);
						pins.push_back(pin);
					}
					else
					{
						pin = hats.at(i).pins.at(j);
						pin.Value = hats.at(i).pins.at(j).DefaultValue;
						pins.push_back(pin);
					}
				}
				else
				{
					pin = hats.at(i).pins.at(j);
					pin.Value = hats.at(i).pins.at(j).DefaultValue;
					pins.push_back(pin);
				}
			}
		}
	}
	return pins;
}
bool HatControllerNodeProcess::hat_present(const eros::device::ConstPtr& t_device)
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
bool HatControllerNodeProcess::load_sensorinfo(std::string name)
{
	bool loaded = false;
	std::string sensor_folder = "/home/robot/config/sensors/" + name;
	std::string sensor_descriptor = "/home/robot/config/sensors/" + name + "/" + name + ".xml";
	TiXmlDocument sensor_doc(sensor_descriptor);
	bool sensorfile_loaded = sensor_doc.LoadFile();
	if(sensorfile_loaded == true)
	{
		if(parse_sensorfile(sensor_doc,name) == true)
		{
			return true;
		}
		else
		{
			printf("Could not parse Sensor File: %s\n",sensor_descriptor.c_str());
			return false;
		}
	}
	else
	{
		printf("Could not load Sensor File: %s\n",sensor_descriptor.c_str());
		return false;
	}
}
bool HatControllerNodeProcess::parse_sensorfile(TiXmlDocument doc,std::string name)
{
	int sensor_index = -1;
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).name == name)
		{
			sensor_index = i;
		}
	}
	if(sensor_index < 0)
	{
		printf("Did not find Sensor: %s\n",name.c_str());
		return false;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
		bool convert = true;
		TiXmlElement *l_pSensorType = l_pRootElement->FirstChildElement( "Type" );
		if(NULL != l_pSensorType)
		{
			sensors.at(sensor_index).type = l_pSensorType->GetText();
		}
		else { printf("Element: Type not found.\n"); return false; }
		if((sensors.at(sensor_index).type == "UltraSonicSensor"))
		{

		}
		else
		{
			printf("Sensor Type: %s Not Supported.\n",sensors.at(sensor_index).type.c_str());
			return false;
		}

		TiXmlElement *l_pOutputDataType = l_pRootElement->FirstChildElement( "OutputDataType" );
		if(NULL != l_pSensorType)
		{
			sensors.at(sensor_index).output_datatype = l_pOutputDataType->GetText();
		}
		else { printf("Element: OutputDataType not found.\n"); return false; }
		if((sensors.at(sensor_index).output_datatype == "signal"))

		{

		}
		else
		{
			printf("Sensor OutputDataType: %s Not Supported.\n",sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}

		TiXmlElement *l_pMinInput = l_pRootElement->FirstChildElement( "MinInputValue" );
		if(NULL != l_pMinInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_inputvalue = std::atof(l_pMinInput->GetText());
		}
		else { printf("Sensor: %s Element: MinInputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pMaxInput = l_pRootElement->FirstChildElement( "MaxInputValue" );
		if(NULL != l_pMaxInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_inputvalue = std::atof(l_pMaxInput->GetText());
		}
		else { printf("Sensor: %s Element: MaxInputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pMinOutput = l_pRootElement->FirstChildElement( "MinOutputValue" );
		if(NULL != l_pMinOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_outputvalue = std::atof(l_pMinOutput->GetText());
		}
		else { printf("Sensor: %s Element: MinOutputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pMaxOutput = l_pRootElement->FirstChildElement( "MaxOutputValue" );
		if(NULL != l_pMaxOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_outputvalue = std::atof(l_pMaxOutput->GetText());
		}
		else { printf("Sensor: %s Element: MaxOutputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pUnits = l_pRootElement->FirstChildElement( "Units" );
		if(NULL != l_pUnits)
		{
			sensors.at(sensor_index).signal.units = l_pUnits->GetText();
		}
		else { printf("Sensor: %s Element: Units not found.\n",sensors.at(sensor_index).name.c_str()); return false; }

		sensors.at(sensor_index).convert = convert;
		if((sensors.at(sensor_index).output_datatype == "signal"))
		{

		}
		else
		{
			printf("Sensor OutputDataType: %s Not Supported.\n",sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}
		sensors.at(sensor_index).initialized = true;
	}
	else {	printf("Element: Sensor not found.\n"); return false; }
	return true;
}
double HatControllerNodeProcess::map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output)
{
	double m = (max_output-min_output)/(max_input-min_input);
	//y-y1 = m*(x-x1);
	double out = (m*(input_value-min_input))+min_output;
	if(max_output >= min_output)
	{
		if(out > max_output) { out = max_output; }
		if(out < min_output) { out = min_output; }
	}
	else// if(min_output > max_output)
	{
		if(out > min_output) { out = min_output; }
		if(out < max_output) { out = max_output; }
	}
	return out;
}
