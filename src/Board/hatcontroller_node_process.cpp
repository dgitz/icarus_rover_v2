#include "hatcontroller_node_process.h"
/*! \brief Constructor
 */
HatControllerNodeProcess::HatControllerNodeProcess()
{
	run_time = 0.0;
	initialized = false;
	ready = false;

	ready_to_arm = false;
	armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	hats.clear();
	hats_running.clear();
	time_sincelast_pps = 0.0;
	pps_counter = 0;
	analyze_timing = false;
	timing_diff.clear();
}
/*! \brief Deconstructor
 */
HatControllerNodeProcess::~HatControllerNodeProcess()
{

}
/*! \brief Initialize Process
 */
icarus_rover_v2::diagnostic HatControllerNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	return diagnostic;
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	time_sincelast_pps+=dt;
	run_time += dt;
	if((mydevice.HatCount == 0) and (mydevice.SensorCount == 0))
	{
		if(initialized == true) { ready = true; }
	}
	bool hats_ready = true;
	bool pps_ok = false;

	//See if hats are ready yet
	for(std::size_t i = 0; i < hats_running.size(); i++)
	{
		if(hats_running.at(i) == true) { hats_ready = hats_ready and true; }
		else { hats_ready = false; }
	}
	if(hats_running.size() == 0) { hats_ready = false; }

	if((pps_counter > 0) and (time_sincelast_pps < 5.0)) { pps_ok = true; }
	else { pps_ok = false; }

	bool status = true;
	if((hats_ready == true))
	{
		status = status and true;
	}
	else
	{
		status = false;
		diag.Level = NOTICE;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"All info for Hats not received yet.");
		diag.Description = std::string(tempstr);
	}

	if((pps_ok == true) || (run_time < 1.0))
	{
		status = status and true;
	}
	else
	{
		status = false;
		diag.Level = WARN;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"PPS Counter: %ld Time since last: %f",pps_counter,time_sincelast_pps);
		diag.Description = std::string(tempstr);
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(initialized == false)
	{
	}
	else
	{
		if(ready == false)
		{
			if(newdevice.DeviceParent == myhostname)
			{
				if(hat_present(newdevice) == true)
				{
					diag.Level = WARN;
					diag.Diagnostic_Type = SOFTWARE;
					diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[512];
					sprintf(tempstr,"Hat: %s already loaded.",
							newdevice.DeviceName.c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				std::size_t hat_message = newdevice.DeviceType.find("Hat");
				if(hat_message != std::string::npos)
				{
					if(newdevice.DeviceType == "ServoHat")
					{
						for(std::size_t i = 0; i < newdevice.pins.size(); i++)
						{
							if((newdevice.pins.at(i).Function == "PWMOutput") or
									(newdevice.pins.at(i).Function == "PWMOutput-NonActuator")) {
							}
							else
							{
								diag.Level = ERROR;
								diag.Diagnostic_Type = SOFTWARE;
								diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
								char tempstr[512];
								sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
										newdevice.DeviceType.c_str(),newdevice.pins.at(i).Function.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}
						}
					}
					else if(newdevice.DeviceType == "TerminalHat")
					{
						for(std::size_t i = 0; i < newdevice.pins.size(); i++)
						{
							if((newdevice.pins.at(i).Function == "DigitalInput") or
									(newdevice.pins.at(i).Function == "DigitalOutput-NonActuator") or
									(newdevice.pins.at(i).Function == "DigitalOutput")) {}
							else
							{
								diag.Level = ERROR;
								diag.Diagnostic_Type = SOFTWARE;
								diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
								char tempstr[512];
								sprintf(tempstr,"Hat Type: %s Pin Function: %s Not supported.",
										newdevice.DeviceType.c_str(),newdevice.pins.at(i).Function.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}
						}
					}
					else if(newdevice.DeviceType == "GPIOHat")
					{
						for(std::size_t i = 0; i < newdevice.pins.size(); i++)
						{
							if(newdevice.pins.at(i).Function == map_PinFunction_ToString(PINMODE_ULTRASONIC_INPUT))
							{
								Sensor new_sensor;
								new_sensor.initialized = false;
								new_sensor.value = 0.0;
								new_sensor.name = newdevice.pins.at(i).ConnectedSensor;
								new_sensor.connected_hat = newdevice;
								new_sensor.connected_pin = newdevice.pins.at(i);
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
										newdevice.DeviceType.c_str(),newdevice.pins.at(i).Function.c_str());
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
						sprintf(tempstr,"Hat Type: %s Not supported.",newdevice.DeviceType.c_str());
						diag.Description = std::string(tempstr);
						return diag;
					}
					hats.push_back(newdevice);
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
/*! \brief Process Command Message
 */
std::vector<icarus_rover_v2::diagnostic> HatControllerNodeProcess::new_commandmsg(icarus_rover_v2::command cmd)
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
std::vector<icarus_rover_v2::diagnostic> HatControllerNodeProcess::check_program_variables()
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_ppsmsg(std_msgs::Bool msg)
{
	time_sincelast_pps = 0.0;
	pps_counter++;
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Received PPS.");
	diag.Description = std::string(tempstr);
	return diag;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_pinmsg(icarus_rover_v2::pin msg)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	int found = -1;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).DeviceName == msg.ParentDevice)
		{
			found = 0;
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if(hats.at(i).pins.at(j).Number == msg.Number)
				{
					found = 1;
					hats.at(i).pins.at(j) = msg;
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
		if(analyze_timing == true)
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			timing_diff.push_back(measure_time_diff(msg.stamp,now));
			if(timing_diff.size() > TIMING_BUFFER_LENGTH)
			{
				timing_diff.erase(timing_diff.begin());
			}
		}

		diag.Level = INFO;
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"Updated Pin %s:%d to value: %d.",msg.ParentDevice.c_str(),msg.Number,msg.Value);
		diag.Description = std::string(tempstr);

	}
	return diag;

}
icarus_rover_v2::device HatControllerNodeProcess::find_hat(uint8_t hatid)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if(hats.at(i).ID == hatid)
		{
			return hats.at(i);
		}
	}
	icarus_rover_v2::device empty_device;
	empty_device.DeviceName = "";
	empty_device.DeviceType = "";
	return empty_device;
}
icarus_rover_v2::pin HatControllerNodeProcess::find_pin(icarus_rover_v2::device hat,std::string pinfunction,uint8_t pinnumber)
{
	for(std::size_t i = 0; i < hat.pins.size(); i++)
	{
		if((hat.pins.at(i).Function == pinfunction) and (hat.pins.at(i).Number == pinnumber))
		{
			return hat.pins.at(i);
		}
	}
	icarus_rover_v2::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_message_GetDIOPort1(uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4)
{
	bool any_error = false;
	uint16_t v[6] = {v1,v2,v3,v4};
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::device hat = find_hat(hatid);
	if(hat.DeviceName == "")
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		sprintf(tempstr,"Hat ID: %d Not Found\n",hatid);
		diag.Description = std::string(tempstr);
		return diag;
	}
	for(uint8_t i = 0; i < 4; i++)
	{
		icarus_rover_v2::pin pin = find_pin(hat,"UltraSonicSensorInput",i);
		if(pin.Name != "")
		{
			if(update_sensor(hat,pin,tov,(double)v[i]) == false)
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
bool HatControllerNodeProcess::update_sensor(icarus_rover_v2::device hat,icarus_rover_v2::pin pin,double tov,double value)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if((sensors.at(i).connected_hat.DeviceName == hat.DeviceName) and
				(sensors.at(i).connected_pin.Name == pin.Name))
		{
			sensors.at(i).tov = tov;
			sensors.at(i).status = SIGNALSTATE_UPDATED;
			if(sensors.at(i).convert == false)
			{
				sensors.at(i).value = value;
			}
			else
			{
				sensors.at(i).value = map_input_to_output(value,sensors.at(i).min_inputvalue,
						sensors.at(i).max_inputvalue,
						sensors.at(i).min_inputvalue,
						sensors.at(i).max_outputvalue);
			}
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::new_pinsmsg(icarus_rover_v2::iopins msg)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	int found = -1;
	for(std::size_t k = 0; k < msg.pins.size(); k++)
	{
		for(std::size_t i = 0; i < hats.size(); i++)
		{
			if(hats.at(i).DeviceName == msg.pins.at(k).ParentDevice)
			{
				found = 0;
				for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
				{
					if(hats.at(i).pins.at(j).Number == msg.pins.at(k).Number)
					{
						found = 1;
						hats.at(i).pins.at(j) = msg.pins.at(k);
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
	sprintf(tempstr,"Updated Pins %d",(int)msg.pins.size());
	diag.Description = std::string(tempstr);
	return diag;

}


icarus_rover_v2::diagnostic HatControllerNodeProcess::new_armedstatemsg(uint8_t msg)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::set_hat_running(std::string devicetype,uint16_t id)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::device hat;
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
		sprintf(tempstr,"Hat: %d Not Found",id);
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
icarus_rover_v2::diagnostic HatControllerNodeProcess::set_terminalhat_initialized()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
std::vector<icarus_rover_v2::pin> HatControllerNodeProcess::get_terminalhatpins(std::string Function)
{
	std::vector<icarus_rover_v2::pin> pins;
	pins.clear();
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "TerminalHat"))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				icarus_rover_v2::pin pin;
				if(hats.at(i).pins.at(j).Function == Function)
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
std::vector<icarus_rover_v2::pin> HatControllerNodeProcess::get_gpiohatpins(uint16_t id)
{
	std::vector<icarus_rover_v2::pin> pins;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "GPIOHat") and (hats.at(i).ID == id))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				icarus_rover_v2::pin pin;
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
std::vector<icarus_rover_v2::pin> HatControllerNodeProcess::get_servohatpins(uint16_t id)
{
	std::vector<icarus_rover_v2::pin> pins;
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceType == "ServoHat") and (hats.at(i).ID == id))
		{
			for(std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				icarus_rover_v2::pin pin;
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
bool HatControllerNodeProcess::hat_present(icarus_rover_v2::device device)
{
	for(std::size_t i = 0; i < hats.size(); i++)
	{
		if((hats.at(i).DeviceName == device.DeviceName))
		{
			return true;
		}
	}
	return false;
}
double HatControllerNodeProcess::measure_time_diff(struct timeval start, struct timeval end)
{
	double t1 = (double)(start.tv_sec)+((double)(start.tv_usec)/1000000.0);
	double t2 = (double)(end.tv_sec)+((double)(end.tv_usec)/1000000.0);
	return t2-t1;
}
double HatControllerNodeProcess::measure_time_diff(ros::Time start, struct timeval end)
{
	double t1 = (double)start.sec + (double)(start.nsec)/1000000000.0;
	double t2 = (double)(end.tv_sec)+((double)(end.tv_usec)/1000000.0);
	return t2-t1;
}
double HatControllerNodeProcess::measure_time_diff(double start, struct timeval end)
{
	double t2 = (double)(end.tv_sec)+((double)(end.tv_usec)/1000000.0);
	return t2-start;
}
double HatControllerNodeProcess::measure_time_diff(struct timeval start, double end)
{
	double t1 = (double)(start.tv_sec)+((double)(start.tv_usec)/1000000.0);
	return end-t1;
}
double HatControllerNodeProcess::measure_time_diff(double start, double end)
{
	return end-start;
}
ros::Time HatControllerNodeProcess::convert_time(struct timeval t_)
{
	ros::Time t;
	t.sec = t_.tv_sec;
	t.nsec = t_.tv_usec*1000;
	return t;
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
			sensors.at(sensor_index).units = l_pUnits->GetText();
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
