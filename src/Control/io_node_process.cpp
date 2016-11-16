#include "io_node_process.h"

IONodeProcess::IONodeProcess()
{
	all_sensor_info_received = false;
	all_device_info_received = false;
	initialize_Ports();
	ms_timer = 0;
	timeout_value_ms = 0;
    init_time = ros::Time::now();
}
IONodeProcess::~IONodeProcess()
{

}
bool IONodeProcess::initialize_Ports()
{
	GPIO_Port.PortName = "GPIO_Port";
	for(int i = 0; i < 32; i++)
	{
		GPIO_Port.Number[i] = i+1;
		GPIO_Port.Mode[i] = PINMODE_UNDEFINED;
		GPIO_Port.Available[i] = false;
		GPIO_Port.Value[i] = 0;
		GPIO_Port.ConnectingDevice.push_back("");
	}

}
icarus_rover_v2::diagnostic IONodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname)
{
	//initialize_stateack_messages();
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
	timeout_value_ms = INITIAL_TIMEOUT_VALUE_MS;
	return diagnostic;
}
icarus_rover_v2::diagnostic IONodeProcess::update(long dt)
{
	ms_timer += dt;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;
		//printf("Mode: %d,%d\n",node_state,board_state);
		/*if((node_state == GPIO_MODE_INITIALIZING) && (board_state == GPIO_MODE_INITIALIZING))
		{
			//printf("Setting to true.\n");
			send_configure_DIO_PortA.trigger = true;
			send_configure_DIO_PortB.trigger = true;
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZED;
		}
		*/

	}
	if(mydevice.Architecture == "armv7l")
	{
		for(int i = 0; i < 32;i++)
		{
			if(GPIO_Port.Available[i] == true)
			{
				if(GPIO_Port.Mode[i] == PINMODE_DIGITAL_INPUT)
				{
					std::string getval_str = "/sys/class/gpio/gpio" +
							boost::lexical_cast<std::string>((int)GPIO_Port.Number[i]) + "/value";
					ifstream getvalgpio(getval_str.c_str());// open value file for gpio
					if (getvalgpio.is_open() == false)
					{
						char tempstr[255];
						diagnostic.Level = ERROR;
						diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						sprintf(tempstr,"Unable to get value of Pin: %d",GPIO_Port.Number[i]);
						diagnostic.Description = tempstr;
						mylogger->log_error(tempstr);
						return diagnostic;
					}
					std::string val;
					getvalgpio >> val ;  //read gpio value

					if(val == "0")
					{
						GPIO_Port.Value[i] = 0;
					}
					else
					{
						GPIO_Port.Value[i] = 1;
					}
					getvalgpio.close(); //close the value file
				}
			}
		}
	}

	//send_nodemode.trigger = true;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}
state_ack IONodeProcess::get_stateack(std::string name)
{
/*
	if(name == send_configure_DIO_PortA.name)
	{
		return send_configure_DIO_PortA;
	}
	else if(name == send_configure_DIO_PortB.name)
	{
		return send_configure_DIO_PortB;
	}
	else if(name == send_testmessage_command.name)
	{
		return send_testmessage_command;
	}
	else if(name == send_nodemode.name)
	{
		return send_nodemode;
	}
	else if(name == send_set_DIO_PortA.name)
	{
		return send_set_DIO_PortA;
	}
	else if(name == send_set_DIO_PortB.name)
	{
		return send_set_DIO_PortB;
	}
	else
	{
		state_ack emptystateack;
		emptystateack.name = "";
		return emptystateack;
	}
	*/
	state_ack emptystateack;
	emptystateack.name = "";
	return emptystateack;
}
bool IONodeProcess::set_stateack(state_ack stateack)
{
	printf("name: %s\n",stateack.name.c_str());
	return false;
	/*
	if(stateack.name == "Send Configure DIO PortA")
	{
		send_configure_DIO_PortA = stateack;
	}
	else if(stateack.name == "Send Configure DIO PortB")
	{
		send_configure_DIO_PortB = stateack;
	}
	else if(stateack.name == "Send Test Message Command")
	{
		send_testmessage_command = stateack;
	}
	else if(stateack.name == "Send Node Mode")
	{
		send_nodemode = stateack;
	}
	else if(stateack.name == "Send Set DIO PortA")
	{
		send_set_DIO_PortA = stateack;

	}
	else if(stateack.name == "Send Set DIO PortB")
	{
		send_set_DIO_PortB = stateack;
	}
	else
	{
		return false;
	}
	return true;
	*/
}
icarus_rover_v2::diagnostic IONodeProcess::new_pinmsg(icarus_rover_v2::pin pinmsg)
{
	if(pinmsg.Port == GPIO_Port.PortName)
	{
		if(GPIO_Port.Available[pinmsg.Number-1] == true)
		{
			GPIO_Port.Value[pinmsg.Number-1] = pinmsg.Value;
			if(mydevice.Architecture == "armv7l")
			{
				//printf("pin: %d\n",pinmsg.Number);
				string setval_str = "/sys/class/gpio/gpio" + boost::lexical_cast<std::string>((int)pinmsg.Number) + "/value";
				//printf("Using set string: %s\n",setval_str.c_str());
				ofstream setvalgpio(setval_str.c_str()); // open value file for gpio
				if (setvalgpio.is_open() == false)
				{
					char tempstr[255];
					sprintf(tempstr,"Unable to set GPIO Pin: %d",pinmsg.Number);
					diagnostic.Diagnostic_Type = SOFTWARE;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					diagnostic.Description = std::string(tempstr);
					mylogger->log_error(tempstr);
				}
				if(pinmsg.Value == 0)
				{
					setvalgpio << "0"; ;//write value to value file
				}
				else
				{
					setvalgpio << "1"; ;//write value to value file
				}
				setvalgpio.close();// close value file
			}
		}
	}
	return diagnostic;
}
bool IONodeProcess::checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers)
{
	return true;
}
icarus_rover_v2::diagnostic IONodeProcess::new_commandmsg(icarus_rover_v2::command msg)
{
	return diagnostic;
}

icarus_rover_v2::diagnostic IONodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;
		mydevice.pins = newdevice.pins;
		for(int i = 0; i < mydevice.pins.size();i++)
		{
			if(configure_pin(mydevice.pins.at(i).Port,mydevice.pins.at(i).Number,mydevice.pins.at(i).Function,mydevice.pins.at(i).ConnectedDevice)==false)
			{
				char tempstr[256];
				sprintf(tempstr,"Couldn't Configure Pin on Port: %s Number: %d Function: %s Connecting Device: %s",
						mydevice.pins.at(i).Port.c_str(),
						mydevice.pins.at(i).Number,
						mydevice.pins.at(i).Function.c_str(),
						mydevice.pins.at(i).ConnectedDevice.c_str());
				//printf("%s\n",tempstr);
				mylogger->log_error(tempstr);
				diagnostic.Diagnostic_Type = SOFTWARE;
				diagnostic.Level = ERROR;
				diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
				diagnostic.Description = std::string(tempstr);
			}
			else
			{
				char tempstr[256];
				sprintf(tempstr,"Configured Pin on Port: %s Number: %d Function: %s Connected Device: %s",
						mydevice.pins.at(i).Port.c_str(),
						mydevice.pins.at(i).Number,
						mydevice.pins.at(i).Function.c_str(),
						mydevice.pins.at(i).ConnectedDevice.c_str());
				//printf("%s\n",tempstr);
				mylogger->log_debug(tempstr);
			}
		}
		all_device_info_received = true;
	}
	return diagnostic;
}
bool IONodeProcess::configure_pin(std::string Port, uint8_t Number, std::string Function,std::string ConnectedDevice)
{
	bool status = true;
	int function = map_PinFunction_ToInt(Function);
	if((Number < 1) || (Number > 32)) { return false; }
	if(function == PINMODE_UNDEFINED) { return false; }
	if(Port == "GPIO_Port")
	{
		GPIO_Port.Number[Number-1] = Number;
		GPIO_Port.Mode[Number-1] = function;
		GPIO_Port.Available[Number-1] = true;
		GPIO_Port.Value[Number-1] = 0;
		GPIO_Port.ConnectingDevice.at(Number-1) = ConnectedDevice;
		if(mydevice.Architecture == "armv7l")
		{
			ofstream export_file;
			export_file.open("/sys/class/gpio/export");
			if (export_file.is_open() == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Unable to export GPIO Pin: %d",Number);
				mylogger->log_fatal(tempstr);
			}
			export_file << Number; //write GPIO number to export
			export_file.close(); //close export file
			ofstream setdirection_file;
			std::string setdirection_str = "/sys/class/gpio/gpio" + boost::lexical_cast<std::string>((int)Number) + "/direction";
			setdirection_file.open(setdirection_str.c_str());
			if (setdirection_file.is_open() == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Unable to Set Direction for GPIO Pin: %d",Number);
				mylogger->log_fatal(tempstr);
			}
			if(function == PINMODE_DIGITAL_INPUT)
			{
				setdirection_file << "in";
			}
			else if(function == PINMODE_DIGITAL_OUTPUT)
			{
				setdirection_file << "out";
			}
			else
			{
				char tempstr[255];
				sprintf(tempstr,"Pin Mode: %s on Pin Number: %d Not Supported.",Function.c_str(),Number);
				mylogger->log_fatal(tempstr);
				return false;
			}
			setdirection_file.close();

		}
	}
	else
	{
		status = false;
	}
	return status;
}

std::string IONodeProcess::map_PinFunction_ToString(int function)
{
	switch(function)
	{
	case PINMODE_UNDEFINED:						return "Undefined";					break;
	case PINMODE_DIGITAL_OUTPUT: 				return "DigitalOutput"; 			break;
	case PINMODE_DIGITAL_INPUT: 				return "DigitalInput"; 				break;
	case PINMODE_ANALOG_INPUT: 					return "AnalogInput"; 				break;
	case PINMODE_FORCESENSOR_INPUT: 			return "ForceSensorInput"; 			break;
	case PINMODE_ULTRASONIC_INPUT: 				return "UltraSonicSensorInput"; 	break;
	case PINMODE_QUADRATUREENCODER_INPUT: 		return "QuadratureEncoderInput";	break;
	case PINMODE_PWM_OUTPUT: 					return "PwmOutput"; 				break;
	default: 									return ""; 							break;
	}
}
int IONodeProcess::map_PinFunction_ToInt(std::string Function)
{
	if(Function == "DigitalInput")				{	return PINMODE_DIGITAL_INPUT;		}
	else if(Function == "DigitalOutput")		{	return PINMODE_DIGITAL_OUTPUT;		}
	else if(Function == "AnalogInput")			{	return PINMODE_ANALOG_INPUT;		}
	else if(Function == "ForceSensorInput")		{	return PINMODE_FORCESENSOR_INPUT;	}
	else if(Function == "UltraSonicSensorInput"){	return PINMODE_ULTRASONIC_INPUT;	}
	else if(Function == "PWMOutput")			{	return PINMODE_PWM_OUTPUT;			}
	else { return PINMODE_UNDEFINED; }
}
Port_Info IONodeProcess::get_PortInfo(std::string PortName)
{
	if(PortName == "GPIO_Port")
	{
		return GPIO_Port;
	}
	else
	{
		Port_Info blank;
		blank.PortName == "";
		return blank;
	}
}
void IONodeProcess::initialize_stateack_messages()
{
	/*
	send_configure_DIO_PortA.name = "Send Configure DIO PortA";
	send_configure_DIO_PortA.trigger = false;
	send_configure_DIO_PortA.state = false;
	gettimeofday(&send_configure_DIO_PortA.orig_send_time,NULL);
	send_configure_DIO_PortA.retries = 0;
	send_configure_DIO_PortA.timeout_counter = 0;
	send_configure_DIO_PortA.retry_mode = false;
	send_configure_DIO_PortA.failed = false;
	send_configure_DIO_PortA.flag1 = 0; //This flag represents the Board Index

	send_configure_DIO_PortB.name = "Send Configure DIO PortB";
	send_configure_DIO_PortB.trigger = false;
	send_configure_DIO_PortB.state = false;
	gettimeofday(&send_configure_DIO_PortB.orig_send_time,NULL);
	send_configure_DIO_PortB.retries = 0;
	send_configure_DIO_PortB.timeout_counter = 0;
	send_configure_DIO_PortB.retry_mode = false;
	send_configure_DIO_PortB.failed = false;
	send_configure_DIO_PortB.flag1 = 0; //This flag represents the Board Index


	send_testmessage_command.name = "Send Test Message Command";
	send_testmessage_command.trigger = false;
	send_testmessage_command.state = false;
	gettimeofday(&send_testmessage_command.orig_send_time,NULL);
	send_testmessage_command.retries = 0;
	send_testmessage_command.timeout_counter = 0;
	send_testmessage_command.retry_mode = false;
	send_testmessage_command.failed = false;

	send_nodemode.name = "Send Node Mode";
	send_nodemode.trigger = false;
	send_nodemode.state = false;
	gettimeofday(&send_nodemode.orig_send_time,NULL);
	send_nodemode.retries = 0;
	send_nodemode.timeout_counter = 0;
	send_nodemode.retry_mode = false;
	send_nodemode.failed = false;

	send_set_DIO_PortA.name = "Send Set DIO PortA";
	send_set_DIO_PortA.trigger = false;
	send_set_DIO_PortA.state = false;
	gettimeofday(&send_set_DIO_PortA.orig_send_time,NULL);
	send_set_DIO_PortA.retries = 0;
	send_set_DIO_PortA.timeout_counter = 0;
	send_set_DIO_PortA.retry_mode = false;
	send_set_DIO_PortA.failed = false;

	send_set_DIO_PortB.name = "Send Set DIO PortB";
	send_set_DIO_PortB.trigger = false;
	send_set_DIO_PortB.state = false;
	gettimeofday(&send_set_DIO_PortB.orig_send_time,NULL);
	send_set_DIO_PortB.retries = 0;
	send_set_DIO_PortB.timeout_counter = 0;
	send_set_DIO_PortB.retry_mode = false;
	send_set_DIO_PortB.failed = false;
	*/

}
double IONodeProcess::time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
