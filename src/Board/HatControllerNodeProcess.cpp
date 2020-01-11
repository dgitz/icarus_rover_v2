#include "HatControllerNodeProcess.h"
bool HatControllerNodeProcess::initialize_supportedhats()
{
	supported_partnumbers.push_back(PN_100003);
	supported_partnumbers.push_back(PN_100007);
	supported_partnumbers.push_back(PN_625004);
	{ //HAT: GPIOHAT PN: 1000007
		HatControllerNodeProcess::HatMap hat;
		hat.FAST_PN = PN_100007;
		hat.DeviceType = DEVICETYPE_GPIOHAT;
		//DIO PORT1 MAX SIZE=4
		hat.PinMap.push_back(create_pindefinition("PC6", PORT_DIGIPORT_1, 0));
		hat.PinMap.push_back(create_pindefinition("PE6", PORT_DIGIPORT_1, 1));
		hat.PinMap.push_back(create_pindefinition("PB4", PORT_DIGIPORT_1, 2));
		hat.PinMap.push_back(create_pindefinition("PB7", PORT_DIGIPORT_1, 3));

		//ANA PORT1 MAX SIZE=6
		hat.PinMap.push_back(create_pindefinition("PF7", PORT_ANAPORT_1, 0));
		hat.PinMap.push_back(create_pindefinition("PF6", PORT_ANAPORT_1, 1));
		hat.PinMap.push_back(create_pindefinition("PF5", PORT_ANAPORT_1, 2));
		hat.PinMap.push_back(create_pindefinition("PF4", PORT_ANAPORT_1, 3));
		hat.PinMap.push_back(create_pindefinition("PF1", PORT_ANAPORT_1, 4));
		hat.PinMap.push_back(create_pindefinition("PF0", PORT_ANAPORT_1, 5));

		//ANA PORT2 MAX SIZE=6
		hat.PinMap.push_back(create_pindefinition("PD4", PORT_ANAPORT_2, 0));
		hat.PinMap.push_back(create_pindefinition("PB5", PORT_ANAPORT_2, 1));
		hat.PinMap.push_back(create_pindefinition("PB6", PORT_ANAPORT_2, 2));
		hat.PinMap.push_back(create_pindefinition("PD6", PORT_ANAPORT_2, 3));
		supported_hats.push_back(hat);
	}
	{ //HAT: SERVOHAT PN: 625004
		HatControllerNodeProcess::HatMap hat;
		hat.FAST_PN = PN_625004;
		hat.DeviceType = DEVICETYPE_SERVOHAT;
		//DIO PORT1 MAX SIZE=16
		hat.PinMap.push_back(create_pindefinition("CH0", PORT_DIGOPORT_1, 0));
		hat.PinMap.push_back(create_pindefinition("CH1", PORT_DIGOPORT_1, 1));
		hat.PinMap.push_back(create_pindefinition("CH2", PORT_DIGOPORT_1, 2));
		hat.PinMap.push_back(create_pindefinition("CH3", PORT_DIGOPORT_1, 3));
		hat.PinMap.push_back(create_pindefinition("CH4", PORT_DIGOPORT_1, 4));
		hat.PinMap.push_back(create_pindefinition("CH5", PORT_DIGOPORT_1, 5));
		hat.PinMap.push_back(create_pindefinition("CH6", PORT_DIGOPORT_1, 6));
		hat.PinMap.push_back(create_pindefinition("CH7", PORT_DIGOPORT_1, 7));
		hat.PinMap.push_back(create_pindefinition("CH8", PORT_DIGOPORT_1, 8));
		hat.PinMap.push_back(create_pindefinition("CH9", PORT_DIGOPORT_1, 9));
		hat.PinMap.push_back(create_pindefinition("CH10", PORT_DIGOPORT_1, 10));
		hat.PinMap.push_back(create_pindefinition("CH11", PORT_DIGOPORT_1, 11));
		hat.PinMap.push_back(create_pindefinition("CH12", PORT_DIGOPORT_1, 12));
		hat.PinMap.push_back(create_pindefinition("CH13", PORT_DIGOPORT_1, 13));
		hat.PinMap.push_back(create_pindefinition("CH14", PORT_DIGOPORT_1, 14));
		hat.PinMap.push_back(create_pindefinition("CH15", PORT_DIGOPORT_1, 15));

		supported_hats.push_back(hat);
	}
	{ //HAT: TERMINALHAT PN: 625005
		HatControllerNodeProcess::HatMap hat;
		hat.FAST_PN = PN_625005;
		hat.DeviceType = DEVICETYPE_TERMINALHAT;
		//DIO PORT1 MAX SIZE=4
		hat.PinMap.push_back(create_pindefinition("GPIO21", PORT_DIGIPORT_1, 0));
		hat.PinMap.push_back(create_pindefinition("GPIO23", PORT_DIGIPORT_1, 1));
		hat.PinMap.push_back(create_pindefinition("GPIO25", PORT_DIGIPORT_1, 2));

		supported_hats.push_back(hat);
	}
	return true;
}
eros::diagnostic HatControllerNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	reset();
	hats.clear();
	hats_running.clear();
	analyze_timing = false;
	bool status = initialize_supportedhats();
	if (status == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Unable to initialize Supported Hats.");
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
	}

	diag = update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "No Error.");
	diag = update_diagnostic(SENSORS, INFO, NOERROR, "No Error.");
	return diag;
}
eros::diagnostic HatControllerNodeProcess::update(double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if (task_state == TASKSTATE_PAUSE)
	{
	}
	else if (task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	else if (task_state == TASKSTATE_INITIALIZED)
	{
	}
	else if (task_state == TASKSTATE_RUNNING)
	{
	}
	else if (task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	diag = update_baseprocess(t_dt, t_ros_time);

	if ((mydevice.HatCount == 0) and (mydevice.SensorCount == 0))
	{
		if (task_state == TASKSTATE_INITIALIZED)
		{
			request_statechange(TASKSTATE_RUNNING);
		}
	}
	bool hats_ready = true;
	for (std::size_t i = 0; i < hats_running.size(); i++)
	{
		if (hats_running.at(i) == true)
		{
			hats_ready = hats_ready and true;
		}
		else
		{
			hats_ready = false;
		}
	}
	if (hats_running.size() == 0)
	{
		hats_ready = false;
	}
	bool status = true;
	if ((hats_ready == true))
	{
		status = status and true;
	}
	else
	{
		status = false;
		diag = update_diagnostic(DATA_STORAGE, INFO, INITIALIZING, "Initializing.");
	}
	ready_to_arm = status;
	if (status == false)
	{
		armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running.");
	}
	return diag;
}
std::vector<eros::diagnostic> HatControllerNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
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
	else if(t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if(node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if(v == false)
			{
				diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
					"Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if(task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE,NOTICE,DIAGNOSTIC_PASSED,
					"Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}

		}
	}
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
	}
	return diaglist;
}
std::vector<eros::diagnostic> HatControllerNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true)
	{
		diag = update_diagnostic(SOFTWARE, INFO, DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, WARN, DIAGNOSTIC_FAILED, "Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
bool HatControllerNodeProcess::sensors_initialized()
{
	bool sensors_init = true;
	if ((sensors.size() == 0) and (mydevice.SensorCount == 0))
	{
		return true;
	}
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if (sensors.at(i).initialized == false)
		{
			sensors_init = false;
		}
	}
	return sensors_init;
}
std::string HatControllerNodeProcess::map_PinFunction_ToString(int function)
{
	switch (function)
	{
	case PINMODE_UNDEFINED:
		return "Undefined";
		break;
	case PINMODE_DIGITAL_OUTPUT:
		return "DigitalOutput";
		break;
	case PINMODE_DIGITAL_INPUT:
		return "DigitalInput";
		break;
	case PINMODE_ANALOG_INPUT:
		return "AnalogInput";
		break;
	case PINMODE_ULTRASONIC_INPUT:
		return "UltraSonicSensorInput";
		break;
	case PINMODE_QUADRATUREENCODER_INPUT:
		return "QuadratureEncoderInput";
		break;
	case PINMODE_PWM_OUTPUT:
		return "PWMOutput";
		break;
	case PINMODE_PWM_OUTPUT_NON_ACTUATOR:
		return "PWMOutput-NonActuator";
		break;
	case PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR:
		return "DigitalOutput-NonActuator";
		break;
	default:
		return "";
		break;
	}
}
int HatControllerNodeProcess::map_PinFunction_ToInt(std::string Function)
{
	if (Function == "DigitalInput")
	{
		return PINMODE_DIGITAL_INPUT;
	}
	else if (Function == "DigitalOutput")
	{
		return PINMODE_DIGITAL_OUTPUT;
	}
	else if (Function == "AnalogInput")
	{
		return PINMODE_ANALOG_INPUT;
	}
	else if (Function == "UltraSonicSensorInput")
	{
		return PINMODE_ULTRASONIC_INPUT;
	}
	else if (Function == "PWMOutput")
	{
		return PINMODE_PWM_OUTPUT;
	}
	else if (Function == "PWMOutput-NonActuator")
	{
		return PINMODE_PWM_OUTPUT_NON_ACTUATOR;
	}
	else if (Function == "DigitalOutput-NonActuator")
	{
		return PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR;
	}
	else
	{
		return PINMODE_UNDEFINED;
	}
}
eros::diagnostic HatControllerNodeProcess::new_devicemsg(const eros::device::ConstPtr &t_newdevice)
{
	eros::diagnostic diag = root_diagnostic;
	eros::device device = convert_fromptr(t_newdevice);

	if (task_state == TASKSTATE_INITIALIZED)
	{
		if (t_newdevice->DeviceParent == host_name)
		{
			if (hat_present(t_newdevice) == true)
			{
				char tempstr[512];
				sprintf(tempstr, "Hat: %s already loaded.",
						t_newdevice->DeviceName.c_str());
				diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
				return diag;
			}
			std::size_t hat_message = t_newdevice->DeviceType.find("Hat");
			if (hat_message != std::string::npos)
			{
				if (t_newdevice->DeviceType == DEVICETYPE_SERVOHAT)
				{
					for (std::size_t i = 0; i < t_newdevice->pins.size(); i++)
					{
						if ((t_newdevice->pins.at(i).Function == "PWMOutput") or
							(t_newdevice->pins.at(i).Function == "PWMOutput-NonActuator"))
						{
						}
						else
						{
							char tempstr[512];
							sprintf(tempstr, "Hat Type: %s Pin Function: %s Not supported.",
									t_newdevice->DeviceType.c_str(), t_newdevice->pins.at(i).Function.c_str());
							diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
							return diag;
						}
					}
				}
				else if (t_newdevice->DeviceType == DEVICETYPE_TERMINALHAT)
				{
					for (std::size_t i = 0; i < t_newdevice->pins.size(); i++)
					{
						if ((t_newdevice->pins.at(i).Function == "DigitalInput") or
							(t_newdevice->pins.at(i).Function == "DigitalInput-Safety") or
							(t_newdevice->pins.at(i).Function == "DigitalOutput-NonActuator") or
							(t_newdevice->pins.at(i).Function == "DigitalOutput"))
						{
						}

						else
						{
							char tempstr[512];
							sprintf(tempstr, "Hat Type: %s Pin Function: %s Not supported.",
									t_newdevice->DeviceType.c_str(), t_newdevice->pins.at(i).Function.c_str());
							diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
							return diag;
						}
					}
				}
				else if (t_newdevice->DeviceType == DEVICETYPE_GPIOHAT)
				{
					for (std::size_t i = 0; i < t_newdevice->pins.size(); i++)
					{
						int function = map_PinFunction_ToInt(t_newdevice->pins.at(i).Function);
						if (function == PINMODE_UNDEFINED)
						{
							char tempstr[512];
							sprintf(tempstr, "Hat Type: %s Pin Function: %s Not supported.",
									t_newdevice->DeviceType.c_str(), t_newdevice->pins.at(i).Function.c_str());
							diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
							return diag;
						}
						else if (function == PINMODE_ULTRASONIC_INPUT)
						{
							Sensor new_sensor;
							new_sensor.initialized = false;
							new_sensor.signal.value = 0.0;
							new_sensor.name = t_newdevice->pins.at(i).ConnectedSensor;
							new_sensor.connected_hat = device;
							new_sensor.connected_pin = t_newdevice->pins.at(i);
							new_sensor.status = SIGNALSTATE_UNDEFINED;
							diag = update_diagnostic(new_sensor.name, SENSORS, INFO, NOERROR, "No Error.");
							sensors.push_back(new_sensor);
							if (load_sensorinfo(new_sensor.name) == false)
							{
								char tempstr[512];
								sprintf(tempstr, "Unable to load info for Sensor: %s", new_sensor.name.c_str());
								diag = update_diagnostic(new_sensor.name, SENSORS, ERROR, INITIALIZING_ERROR, std::string(tempstr));
								return diag;
							}
						}
						else if ((function == PINMODE_DIGITAL_OUTPUT) ||
								 (function == PINMODE_DIGITAL_INPUT) ||
								 (function == PINMODE_ANALOG_INPUT) ||
								 (function == PINMODE_QUADRATUREENCODER_INPUT) ||
								 (function == PINMODE_PWM_OUTPUT) ||
								 (function == PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR) ||
								 (function == PINMODE_PWM_OUTPUT_NON_ACTUATOR) ||
								 (function == PINMODE_ARMCOMMAND_OUTPUT) ||
								 (function == PINMODE_ARMCOMMAND_INPUT) ||
								 (function == PINMODE_NOCHANGE))
						{
						}
						else
						{
							char tempstr[512];
							sprintf(tempstr, "Hat Type: %s Pin Function: %s Not supported.",
									t_newdevice->DeviceType.c_str(), t_newdevice->pins.at(i).Function.c_str());
							diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
							return diag;
						}
					}
				}
				else
				{
					char tempstr[512];
					sprintf(tempstr, "Hat Type: %s Not supported.", t_newdevice->DeviceType.c_str());
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}
				diag = update_diagnostic(device.DeviceName, COMMUNICATIONS, NOTICE, NOERROR, "No Error.");
				hats.push_back(device);
				hats_running.push_back(false);
			}
		}
	}
	if (((int)hats.size() == mydevice.HatCount) and
		(sensors_initialized() == true))
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
		request_statechange(TASKSTATE_RUNNING);
	}
	return diag;
}

eros::diagnostic HatControllerNodeProcess::new_pinmsg(const eros::pin::ConstPtr &t_msg)
{
	eros::diagnostic diag = root_diagnostic;
	int found = -1;
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if (hats.at(i).DeviceName == t_msg->ParentDevice)
		{
			found = 0;
			for (std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if (hats.at(i).pins.at(j).Name == t_msg->Name)
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

	if (found == 0)
	{
		char tempstr[512];
		sprintf(tempstr, "Pin Msg for My Hat but Pin not found: %s:%s", t_msg->ParentDevice.c_str(), t_msg->Name.c_str());
		diag = update_diagnostic(t_msg->ParentDevice, COMMUNICATIONS, WARN, DEVICE_NOT_AVAILABLE, std::string(tempstr));
	}
	else
	{
		if (analyze_timing == true)
		{
			timing_diff.push_back(t_msg->stamp.toSec() - run_time);
			if (timing_diff.size() > TIMING_BUFFER_LENGTH)
			{
				timing_diff.erase(timing_diff.begin());
			}
		}
		char tempstr[512];
		sprintf(tempstr, "Updated Pin %s:%s to value: %d.", t_msg->ParentDevice.c_str(), t_msg->Name.c_str(), t_msg->Value);
		diag = update_diagnostic(t_msg->ParentDevice, COMMUNICATIONS, INFO, NOERROR, std::string(tempstr));
	}
	return diag;
}
eros::device HatControllerNodeProcess::find_hat(uint8_t hatid)
{
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if (hats.at(i).ID == hatid)
		{
			return hats.at(i);
		}
	}
	eros::device empty_device;
	empty_device.DeviceName = "";
	empty_device.DeviceType = "";
	return empty_device;
}
eros::pin HatControllerNodeProcess::find_pin(const eros::device::ConstPtr &t_device, uint8_t port_id, uint8_t port_pinnumber)
{
	for (std::size_t i = 0; i < supported_hats.size(); ++i)
	{
		if (t_device->PartNumber == supported_hats.at(i).FAST_PN)
		{
			for (std::size_t j = 0; j < supported_hats.at(i).PinMap.size(); ++j)
			{
				if ((supported_hats.at(i).PinMap.at(j).port_id == port_id) &&
					(supported_hats.at(i).PinMap.at(j).port_pinnumber == port_pinnumber))
				{
					return find_pin(t_device, supported_hats.at(i).PinMap.at(j).PinName);
				}
			}
		}
	}
	eros::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
eros::pin HatControllerNodeProcess::find_pin(const eros::device::ConstPtr &hat, std::string pin_name)
{
	for (std::size_t i = 0; i < hat->pins.size(); i++)
	{
		if ((hat->pins.at(i).Name == pin_name))
		{
			return hat->pins.at(i);
		}
	}
	eros::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
eros::diagnostic HatControllerNodeProcess::new_message_GetANAPort1(std::string device_type, uint8_t hatid, double tov, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4, uint16_t v5, uint16_t v6)
{
	uint16_t port_width = 6;
	eros::diagnostic diag = root_diagnostic;
	if ((device_type == DEVICETYPE_GPIOHAT))
	{
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, "DeviceType: " + device_type + " Not Supported.");
		return diag;
	}
	uint16_t v[port_width] = {v1, v2, v3, v4, v5, v6};

	eros::device hat = find_hat(hatid);
	if (hat.DeviceName == "")
	{
		char tempstr[1024];
		sprintf(tempstr, "Hat ID: %d Not Found.  Defined Hats: ", hatid);
		char tempstr2[512];
		for (std::size_t j = 0; j < hats.size(); ++j)
		{
			sprintf(tempstr2, "%s:%ld,", hats.at(j).DeviceName.c_str(), hats.at(j).ID);
		}
		sprintf(tempstr, "%s%s\n", tempstr, tempstr2);
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
		return diag;
	}
	for (uint8_t i = 0; i < port_width; i++)
	{
		eros::device::ConstPtr hat_ptr(new eros::device(hat));
		eros::pin pin = find_pin(hat_ptr, PORT_ANAPORT_1, i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if (pin.ConnectedSensor != "")
		{
			if (update_sensor(hat_ptr, pin_ptr, tov, (double)v[i]) == false)
			{
				diag = update_diagnostic(hat_ptr->DeviceName, COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to Update Pin: " + pin.ConnectedSensor);
			}
		}
		if (pin.Name != "")
		{
			pin.Value = v[i];
			diag = update_pin(device_type, hatid, pin.Name, pin);
			diag = update_diagnostic(diag);
		}
	}
	return diag;
}
eros::diagnostic HatControllerNodeProcess::new_message_GetANAPort2(std::string device_type, uint8_t hatid, double tov, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4, uint16_t v5, uint16_t v6)
{
	uint16_t port_width = 6;
	eros::diagnostic diag = root_diagnostic;
	if ((device_type == DEVICETYPE_GPIOHAT))
	{
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, "DeviceType: " + device_type + " Not Supported.");
		return diag;
	}
	uint16_t v[port_width] = {v1, v2, v3, v4, v5, v6};

	eros::device hat = find_hat(hatid);
	if (hat.DeviceName == "")
	{
		char tempstr[1024];
		sprintf(tempstr, "Hat ID: %d Not Found.  Defined Hats: ", hatid);
		char tempstr2[512];
		for (std::size_t j = 0; j < hats.size(); ++j)
		{
			sprintf(tempstr2, "%s:%ld,", hats.at(j).DeviceName.c_str(), hats.at(j).ID);
		}
		sprintf(tempstr, "%s%s\n", tempstr, tempstr2);
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
		return diag;
	}
	for (uint8_t i = 0; i < port_width; i++)
	{
		eros::device::ConstPtr hat_ptr(new eros::device(hat));
		eros::pin pin = find_pin(hat_ptr, PORT_ANAPORT_2, i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if (pin.ConnectedSensor != "")
		{
			if (update_sensor(hat_ptr, pin_ptr, tov, (double)v[i]) == false)
			{
				diag = update_diagnostic(hat_ptr->DeviceName, COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to Update Pin: " + pin.ConnectedSensor);
			}
		}
		if (pin.Name != "")
		{
			pin.Value = v[i];
			diag = update_pin(device_type, hatid, pin.Name, pin);
			diag = update_diagnostic(diag);
		}
	}
	return diag;
}
eros::diagnostic HatControllerNodeProcess::new_message_GetDIOPort1(std::string device_type, uint8_t hatid, double tov, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4)
{
	uint16_t port_width = 4;
	eros::diagnostic diag = root_diagnostic;
	if ((device_type == DEVICETYPE_GPIOHAT))
	{
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, "DeviceType: " + device_type + " Not Supported.");
		return diag;
	}
	uint16_t v[port_width] = {v1, v2, v3, v4};

	eros::device hat = find_hat(hatid);
	if (hat.DeviceName == "")
	{
		char tempstr[1024];
		sprintf(tempstr, "Hat ID: %d Not Found.  Defined Hats: ", hatid);
		char tempstr2[512];
		for (std::size_t j = 0; j < hats.size(); ++j)
		{
			sprintf(tempstr2, "%s:%ld,", hats.at(j).DeviceName.c_str(), hats.at(j).ID);
		}
		sprintf(tempstr, "%s%s\n", tempstr, tempstr2);
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
		return diag;
	}
	for (uint8_t i = 0; i < port_width; ++i)
	{
		eros::device::ConstPtr hat_ptr(new eros::device(hat));
		eros::pin pin = find_pin(hat_ptr, PORT_DIGIPORT_1, i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if (pin.ConnectedSensor != "")
		{
			if (update_sensor(hat_ptr, pin_ptr, tov, (double)v[i]) == false)
			{
				diag = update_diagnostic(hat_ptr->DeviceName, COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to Update Pin: " + pin.ConnectedSensor);
			}
		}
		pin.Value = v[i];
		diag = update_pin(device_type, hatid, pin.Name, pin);
		diag = update_diagnostic(diag);
	}
	return diag;
}
eros::diagnostic HatControllerNodeProcess::update_pin(std::string device_type, uint8_t device_id, std::string pin_name, eros::pin new_pin)
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for (std::size_t i = 0; i < hats.size(); ++i)
	{
		if ((hats.at(i).DeviceType == device_type) && (hats.at(i).ID == device_id))
		{
			for (std::size_t j = 0; j < hats.at(i).pins.size(); ++j)
			{
				if (hats.at(i).pins.at(j).Name == pin_name)
				{
					found = true;
					hats.at(i).pins.at(j) = new_pin;
				}
			}
		}
	}
	if (found == false)
	{
		diag = update_diagnostic(COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to find Pin " + pin_name + " in Hat: " + device_type + " ID: " + std::to_string(device_id));
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "Updated Pin: " + pin_name);
	}
	return diag;
}
bool HatControllerNodeProcess::update_sensor(const eros::device::ConstPtr &t_device, eros::pin::ConstPtr &t_pin, double tov, double value)
{
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if ((sensors.at(i).connected_hat.DeviceName == t_device->DeviceName) and
			(sensors.at(i).connected_pin.Name == t_pin->Name))
		{
			sensors.at(i).signal.tov = tov;
			sensors.at(i).signal.status = SIGNALSTATE_UPDATED;
			if (sensors.at(i).convert == false)
			{
				sensors.at(i).signal.value = value * sensors.at(i).conversion_factor;
			}
			else
			{
				sensors.at(i).signal.value = map_input_to_output(value, sensors.at(i).min_inputvalue * sensors.at(i).conversion_factor,
																 sensors.at(i).max_inputvalue * sensors.at(i).conversion_factor,
																 sensors.at(i).min_inputvalue * sensors.at(i).conversion_factor,
																 sensors.at(i).max_outputvalue * sensors.at(i).conversion_factor);
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
	if (temp.size() <= (TIMING_BUFFER_LENGTH / 2))
	{
		return 0.0;
	}
	for (std::size_t i = 0; i < temp.size(); i++)
	{
		t += temp.at(i);
	}
	return t / (double)(temp.size());
}
eros::diagnostic HatControllerNodeProcess::new_armedstatemsg(uint8_t msg)
{
	eros::diagnostic diag = root_diagnostic;
	armed_state = msg;
	char tempstr[512];
	sprintf(tempstr, "Rover Armed State: %d Processed.", msg);
	diag = update_diagnostic(SOFTWARE, INFO, NOERROR, std::string(tempstr));
	return diag;
}
bool HatControllerNodeProcess::is_hat_running(std::string devicetype, uint16_t id)
{
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
			if (hats_running.at(i) == true)
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
eros::diagnostic HatControllerNodeProcess::set_hat_running(std::string devicetype, uint16_t id)
{
	eros::diagnostic diag = root_diagnostic;
	eros::device hat;
	bool found = false;
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == devicetype) and (hats.at(i).ID == id))
		{
			found = true;
			hats_running.at(i) = true;
			hat = hats.at(i);
		}
	}
	if (found == false)
	{
		char tempstr[512];
		sprintf(tempstr, "Hat ID: %d Not Found.  Defined Hats: ", id);
		char tempstr2[512];
		for (std::size_t j = 0; j < hats.size(); ++j)
		{
			sprintf(tempstr2, "%s:%ld,", hats.at(j).DeviceName.c_str(), hats.at(j).ID);
		}
		sprintf(tempstr, "%s%s\n", tempstr, tempstr2);
		diag = update_diagnostic(DATA_STORAGE, WARN, INITIALIZING_ERROR, std::string(tempstr));
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr, "%s is now Initialized", hat.DeviceName.c_str());
		diag = update_diagnostic(hat.DeviceName, DATA_STORAGE, INFO, NOERROR, std::string(tempstr));
	}
	return diag;
}
eros::diagnostic HatControllerNodeProcess::set_terminalhat_initialized()
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for (std::size_t i = 0; i < hats.size(); ++i)
	{
		if ((hats.at(i).DeviceType == DEVICETYPE_TERMINALHAT))
		{
			found = true;
			hats_running.at(i) = true;
			break; //Only 1 Terminal Hat supported.
		}
	}
	if (found == false)
	{
		char tempstr[512];
		sprintf(tempstr, "%s Not Found", DEVICETYPE_TERMINALHAT);
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr, "%s is now Initialized", DEVICETYPE_TERMINALHAT);
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, std::string(tempstr));
	}
	return diag;
}
std::vector<uint16_t> HatControllerNodeProcess::get_servohataddresses()
{
	std::vector<uint16_t> addresses;
	addresses.clear();
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if (hats.at(i).DeviceType == DEVICETYPE_SERVOHAT)
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
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if (hats.at(i).DeviceType == DEVICETYPE_GPIOHAT)
		{
			addresses.push_back(hats.at(i).ID);
		}
	}
	return addresses;
}
bool HatControllerNodeProcess::set_terminalhatpinvalue(std::string name, int v)
{
	bool found = false;
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == DEVICETYPE_TERMINALHAT))
		{
			for (std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				if (hats.at(i).pins.at(j).Name == name)
				{
					hats.at(i).pins.at(j).Value = v;
					found = true;
				}
			}
		}
	}
	return found;
}
std::vector<eros::pin> HatControllerNodeProcess::get_terminalhatpins()
{
	std::vector<eros::pin> pins;
	pins.clear();
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == DEVICETYPE_TERMINALHAT))
		{
			return hats.at(i).pins;
		}
	}
	return pins;
}
std::vector<eros::pin> HatControllerNodeProcess::get_terminalhatpins(std::string Function, bool match_exact)
{
	std::vector<eros::pin> pins;
	pins.clear();
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == DEVICETYPE_TERMINALHAT))
		{
			for (std::size_t j = 0; j < hats.at(i).pins.size(); j++)
			{
				eros::pin pin;
				if (((match_exact == true) and (hats.at(i).pins.at(j).Function == Function)) or
					((match_exact == false) and (hats.at(i).pins.at(j).Function.find(Function) != std::string::npos)))
				{
					pin = hats.at(i).pins.at(j);
					if ((Function == "DigitalOutput") and (armed_state != ARMEDSTATUS_ARMED))
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
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == DEVICETYPE_GPIOHAT) and (hats.at(i).ID == id))
		{
			return hats.at(i).pins;
		}
	}
	return pins;
}
std::vector<eros::pin> HatControllerNodeProcess::get_servohatpins(uint16_t id)
{
	std::vector<eros::pin> pins;
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceType == DEVICETYPE_SERVOHAT) and (hats.at(i).ID == id))
		{
			return hats.at(i).pins;
		}
	}
	return pins;
}
bool HatControllerNodeProcess::hat_present(const eros::device::ConstPtr &t_device)
{
	for (std::size_t i = 0; i < hats.size(); i++)
	{
		if ((hats.at(i).DeviceName == t_device->DeviceName))
		{
			return true;
		}
	}
	return false;
}
std::vector<eros::pin> HatControllerNodeProcess::get_pins_byport(std::string devicetype, uint16_t device_id, uint8_t port_id)
{
	std::vector<eros::pin> pins;
	pins.clear();
	for (std::size_t i = 0; i < hats.size(); ++i)
	{
		if ((hats.at(i).DeviceType == devicetype) && (hats.at(i).ID == device_id))
		{
			HatControllerNodeProcess::HatMap hat_map = get_hatmap_bypartnumber(hats.at(i).PartNumber);
			for (std::size_t j = 0; j < hat_map.PinMap.size(); ++j)
			{
				if (hat_map.PinMap.at(j).port_id == port_id)
				{
					eros::device::ConstPtr hat_ptr(new eros::device(hats.at(i)));
					eros::pin pin = find_pin(hat_ptr, hat_map.PinMap.at(j).PinName);
					pins.push_back(pin);
				}
			}
		}
	}
	return pins;
}
HatControllerNodeProcess::HatMap HatControllerNodeProcess::get_hatmap_bypartnumber(std::string partnumber)
{
	for (std::size_t i = 0; i < supported_hats.size(); ++i)
	{
		if (supported_hats.at(i).FAST_PN == partnumber)
		{
			return supported_hats.at(i);
		}
	}
	HatMap empty_map;
	return empty_map;
}
bool HatControllerNodeProcess::load_sensorinfo(std::string name)
{
	std::string sensor_folder = "/home/robot/config/sensors/" + name;
	std::string sensor_descriptor = "/home/robot/config/sensors/" + name + "/" + name + ".xml";
	TiXmlDocument sensor_doc(sensor_descriptor);
	bool sensorfile_loaded = sensor_doc.LoadFile();
	if (sensorfile_loaded == true)
	{
		if (parse_sensorfile(sensor_doc, name) == true)
		{
			return true;
		}
		else
		{
			printf("Could not parse Sensor File: %s\n", sensor_descriptor.c_str());
			return false;
		}
	}
	else
	{
		printf("Could not load Sensor File: %s\n", sensor_descriptor.c_str());
		return false;
	}
}
bool HatControllerNodeProcess::parse_sensorfile(TiXmlDocument doc, std::string name)
{
	int sensor_index = -1;
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if (sensors.at(i).name == name)
		{
			sensor_index = i;
		}
	}
	if (sensor_index < 0)
	{
		printf("Did not find Sensor: %s\n", name.c_str());
		return false;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();

	if (NULL != l_pRootElement)
	{
		bool convert = true;
		TiXmlElement *l_pSensorType = l_pRootElement->FirstChildElement("Type");
		if (NULL != l_pSensorType)
		{
			sensors.at(sensor_index).type = l_pSensorType->GetText();
		}
		else
		{
			printf("Element: Type not found.\n");
			return false;
		}
		if ((sensors.at(sensor_index).type == "UltraSonicSensor"))
		{
		}
		else
		{
			printf("Sensor Type: %s Not Supported.\n", sensors.at(sensor_index).type.c_str());
			return false;
		}

		TiXmlElement *l_pOutputDataType = l_pRootElement->FirstChildElement("OutputDataType");
		if (NULL != l_pSensorType)
		{
			sensors.at(sensor_index).output_datatype = l_pOutputDataType->GetText();
		}
		else
		{
			printf("Element: OutputDataType not found.\n");
			return false;
		}
		if ((sensors.at(sensor_index).output_datatype == "signal"))

		{
		}
		else
		{
			printf("Sensor OutputDataType: %s Not Supported.\n", sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}

		TiXmlElement *l_pMinInput = l_pRootElement->FirstChildElement("MinInputValue");
		if (NULL != l_pMinInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_inputvalue = std::atof(l_pMinInput->GetText());
		}
		else
		{
			printf("Sensor: %s Element: MinInputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pMaxInput = l_pRootElement->FirstChildElement("MaxInputValue");
		if (NULL != l_pMaxInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_inputvalue = std::atof(l_pMaxInput->GetText());
		}
		else
		{
			printf("Sensor: %s Element: MaxInputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pMinOutput = l_pRootElement->FirstChildElement("MinOutputValue");
		if (NULL != l_pMinOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_outputvalue = std::atof(l_pMinOutput->GetText());
		}
		else
		{
			printf("Sensor: %s Element: MinOutputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pMaxOutput = l_pRootElement->FirstChildElement("MaxOutputValue");
		if (NULL != l_pMaxOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_outputvalue = std::atof(l_pMaxOutput->GetText());
		}
		else
		{
			printf("Sensor: %s Element: MaxOutputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pUnits = l_pRootElement->FirstChildElement("Units");
		if (NULL != l_pUnits)
		{
			double conversion_factor = 1.0;
			sensors.at(sensor_index).signal.type = convert_signaltype(l_pUnits->GetText(), &conversion_factor);
			sensors.at(sensor_index).conversion_factor = conversion_factor;
		}
		else
		{
			printf("Sensor: %s Element: Units not found.\n", sensors.at(sensor_index).name.c_str());
			return false;
		}

		sensors.at(sensor_index).convert = convert;
		if ((sensors.at(sensor_index).output_datatype == "signal"))
		{
		}
		else
		{
			printf("Sensor OutputDataType: %s Not Supported.\n", sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}
		sensors.at(sensor_index).initialized = true;
	}
	else
	{
		printf("Element: Sensor not found.\n");
		return false;
	}
	return true;
}
double HatControllerNodeProcess::map_input_to_output(double input_value, double min_input, double max_input, double min_output, double max_output)
{
	double m = (max_output - min_output) / (max_input - min_input);
	//y-y1 = m*(x-x1);
	double out = (m * (input_value - min_input)) + min_output;
	if (max_output >= min_output)
	{
		if (out > max_output)
		{
			out = max_output;
		}
		if (out < min_output)
		{
			out = min_output;
		}
	}
	else // if(min_output > max_output)
	{
		if (out > min_output)
		{
			out = min_output;
		}
		if (out < max_output)
		{
			out = max_output;
		}
	}
	return out;
}