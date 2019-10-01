#include "hatcontroller_node.h"
bool kill_node = false;
bool HatControllerNode::start(int argc,char **argv)
{
	bool status = false;
	process = new HatControllerNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
	diagnostic = preinitialize_basenode(argc,argv);
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	process->initialize(get_basenodename(),get_nodename(),get_hostname(),DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
	{
		diagnostic = process->update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Configured.  Initializing.");
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic HatControllerNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic HatControllerNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	std::string param_analyze_timing = node_name + "/analyze_timing";
	bool analyze_timing;
	if(n->getParam(param_analyze_timing,analyze_timing) == false)
	{
		get_logger()->log_notice("Missing parameter: analyze_timing.  Not analyzing timing.");
	}
	else
	{
		if(analyze_timing == true)
		{
			get_logger()->log_notice("Analyzing Timing.");
		}
		process->set_analyzetiming(analyze_timing);
	}
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&HatControllerNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&HatControllerNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string sensor_spec_path;
	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,&HatControllerNode::ArmedState_Callback,this);

	last_digitaloutput_time = ros::Time::now();
	last_pwmoutput_sub_time = ros::Time::now();

	ServoHats.clear();
	GPIOHats.clear();
	return diagnostic;
}
bool HatControllerNode::run_001hz()
{
	return true;
}
bool HatControllerNode::run_01hz()
{
	eros::diagnostic diag = diagnostic;
	if(process->is_ready())
	{
		std::vector<uint16_t> ServoHats_ids = process->get_servohataddresses();
		for(std::size_t i = 0; i < ServoHats_ids.size(); i++)
		{
			if(process->is_hat_running("ServoHat",ServoHats_ids.at(i)) == false)
			{

				ServoHatDriver servohat;
				ServoHats.push_back(servohat);
				int status = ServoHats.at(i).init(ServoHats_ids.at(i));
				if(status < 0)
				{
					char tempstr[512];
					sprintf(tempstr,"Unable to Start ServoHat at Address: %d",ServoHats_ids.at(i));
					diag = process->update_diagnostic(COMMUNICATIONS,ERROR,INITIALIZING_ERROR,std::string(tempstr));
					get_logger()->log_diagnostic(diag);
					return false;
				}
				diag = process->set_hat_running("ServoHat",ServoHats_ids.at(i));

				if(diag.Level > NOTICE)
				{
					diagnostic_pub.publish(diag);
					get_logger()->log_diagnostic(diag);
				}
			}
		}
		std::vector<uint16_t> GPIOHats_ids = process->get_gpiohataddresses();
		for(std::size_t i = 0; i < GPIOHats_ids.size(); i++)
		{
			if(process->is_hat_running("GPIOHat",GPIOHats_ids.at(i)) == false)
			{

				GPIOHatDriver gpiohat;
				GPIOHats.push_back(gpiohat);
				int status = GPIOHats.at(i).init(GPIOHats_ids.at(i));
				if(status < 0)
				{
					char tempstr[512];
					sprintf(tempstr,"Unable to Start GPIOHat at Address: %d",GPIOHats_ids.at(i));
					diag = process->update_diagnostic(COMMUNICATIONS,ERROR,INITIALIZING_ERROR,std::string(tempstr));
					diagnostic_pub.publish(diag);
					get_logger()->log_diagnostic(diag);
					return false;
				}
				diag = process->set_hat_running("GPIOHat",GPIOHats_ids.at(i));
				if(diag.Level > NOTICE)
				{
					diagnostic_pub.publish(diag);
					get_logger()->log_diagnostic(diag);
				}
			}
		}
		if(process->is_hat_running("TerminalHat",0) == false)
		{
			TerminalHat.init();
			{
				bool any_error = false;
				std::vector<eros::pin> pins = process->get_terminalhatpins("",true);
				for(std::size_t i = 0; i < pins.size(); i++)
				{
					if(TerminalHat.configure_pin(pins.at(i).Name,pins.at(i).Function) == false)
					{
						any_error = true;
						char tempstr[512];
						sprintf(tempstr,"[TerminalHat] Could not configure Pin: %s with Function: %s",pins.at(i).Name.c_str(),pins.at(i).Function.c_str());
						diag = process->update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
						get_logger()->log_error(std::string(tempstr));
						kill_node = 1;
					}
				}
				if(any_error == false)
				{
					diag = process->set_hat_running("TerminalHat",0);
					get_logger()->log_diagnostic(diag);
					if(diag.Level > NOTICE) { diagnostic_pub.publish(diag); }
				}
			}

		}
	}
	return true;
}
bool HatControllerNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool HatControllerNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if(process->is_initialized() == false)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
			else
			{
			}
		}
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
		{
			eros::srv_device srv;
			srv.request.query = "DeviceType=ServoHat";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
			}
			srv.request.query = "DeviceType=GPIOHat";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
			}
			else
			{
			}
			srv.request.query = "DeviceType=TerminalHat";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
			}
			else
			{
			}

		}
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level == WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool HatControllerNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	if(process->is_ready())
	{
		std::vector<uint16_t> ServoHats_ids = process->get_servohataddresses();
		for(std::size_t i = 0; i < ServoHats_ids.size(); i++)
		{
			if(process->is_hat_running("ServoHat",ServoHats_ids.at(i)) == true)
			{
				if(ServoHats_ids.at(i) == ServoHats.at(i).get_address())
				{
					std::vector<eros::pin> pins = process->get_servohatpins(ServoHats_ids.at(i));
					for(std::size_t j = 0; j < pins.size(); j++)
					{
						ServoHats.at(i).setServoValue(pins.at(j).Name, pins.at(j).Value);
					}
				}
				else
				{
					diag.Diagnostic_Type = SOFTWARE;
					diag.Level = ERROR;
					diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
					char tempstr[512];
					sprintf(tempstr,"Trying to Set ServoHat Address: %d but got: %d",ServoHats_ids.at(i),ServoHats.at(i).get_address());
					diag.Description = std::string(tempstr);
					get_logger()->log_diagnostic(diag);
				}
			}

		}
		std::vector<uint16_t> GPIOHats_ids = process->get_gpiohataddresses();
		for(std::size_t i = 0; i < GPIOHats_ids.size(); i++)
		{
			if(process->is_hat_running("GPIOHat",GPIOHats_ids.at(i)) == true)
			{
				if(GPIOHats_ids.at(i) == GPIOHats.at(i).get_address())
				{
					std::vector<eros::pin> pins = process->get_gpiohatpins(GPIOHats_ids.at(i));

				}
				else
				{
					diag.Diagnostic_Type = SOFTWARE;
					diag.Level = ERROR;
					diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
					char tempstr[512];
					sprintf(tempstr,"Trying to Set GPIOHat Address: %d but got: %d",GPIOHats_ids.at(i),GPIOHats.at(i).get_address());
					diag.Description = std::string(tempstr);
					get_logger()->log_diagnostic(diag);
				}
			}

		}
		if(process->is_hat_running("TerminalHat",0) == true)
		{
			bool any_error = false;
			std::vector<eros::pin> pins = process->get_terminalhatpins("DigitalInput",true);
			for(std::size_t i = 0; i < pins.size(); i++)
			{
				if(TerminalHat.configure_pin(pins.at(i).Name,pins.at(i).Function) == false)
				{
					any_error = true;
					char tempstr[512];
					sprintf(tempstr,"[TerminalHat] Could not configure Pin: %s with Function: %s",pins.at(i).Name.c_str(),pins.at(i).Function.c_str());
					diag = process->update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
					get_logger()->log_error(std::string(tempstr));
					kill_node = 1;
				}
			}
		}
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level > WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool HatControllerNode::run_loop1()
{
	eros::diagnostic diag = diagnostic;
	for(std::size_t i = 0; i < GPIOHats.size(); i++)
	{
		unsigned char inputbuffer[12];
		{
			int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_DIO_Port1_ID,inputbuffer);
			if(passed_checksum_calc > 0)
			{
				uint16_t a1,a2,a3,a4;
				int length;
				bool success = i2cmessagehandler->decode_Get_DIO_Port1I2C(inputbuffer,&length,&a1,&a2,&a3,&a4);
				if(success == true)
				{
					diag = process->new_message_GetDIOPort1("GPIOHat",GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						get_logger()->log_diagnostic(diag);
					}
				}
			}
		}
		{
			int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_ANA_Port1_ID,inputbuffer);
			if(passed_checksum_calc > 0)
			{
				uint16_t a1,a2,a3,a4,a5,a6;
				int length;
				bool success = i2cmessagehandler->decode_Get_ANA_Port1I2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == true)
				{
					diag = process->new_message_GetANAPort1("GPIOHat",GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						get_logger()->log_diagnostic(diag);
					}
					
				}
			}
		}
		{
			int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_ANA_Port2_ID,inputbuffer);
			if(passed_checksum_calc > 0)
			{
				uint16_t a1,a2,a3,a4,a5,a6;
				int length;
				bool success = i2cmessagehandler->decode_Get_ANA_Port2I2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == true)
				{
					diag = process->new_message_GetANAPort2("GPIOHat",GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						get_logger()->log_diagnostic(diag);
					}
					
				}
			}
		}
		std::vector<HatControllerNodeProcess::Sensor> sensors = process->get_sensordata();
		for(std::size_t i = 0; i < sensors.size(); ++i)
		{
			signal_sensor_pubs.at(i).publish(sensors.at(i).signal);
		}

	}
	{
		if(process->is_hat_running("TerminalHat",0) == true)
		{
			bool any_error = false;
			{
				std::vector<eros::pin> pins = process->get_terminalhatpins("DigitalInput",true);
				for(std::size_t i = 0; i < pins.size(); i++)
				{
					if(process->set_terminalhatpinvalue(pins.at(i).Name,TerminalHat.read_pin(pins.at(i).Name)) == false)
					{
						any_error = true;
						char tempstr[512];
						sprintf(tempstr,"[TerminalHat] Could not read Pin: %s with Function: %s",pins.at(i).Name.c_str(),pins.at(i).Function.c_str());
						diag = process->update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
						get_logger()->log_diagnostic(diag);
						kill_node = 1;
					}
				}
			}
			{
				std::vector<eros::pin> pins = process->get_terminalhatpins("DigitalOutput",false);

				for(std::size_t i = 0; i < pins.size(); i++)
				{
					if(TerminalHat.set_pin(pins.at(i).Name,pins.at(i).Value) == false)
					{
						any_error = true;
						char tempstr[512];
						sprintf(tempstr,"[TerminalHat] Could not set Pin: %s with Function: %s",pins.at(i).Name.c_str(),pins.at(i).Function.c_str());
						diag = process->update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
						get_logger()->log_diagnostic(diag);
						kill_node = 1;
					}
				}
			}


		}
		{
			std::vector<eros::pin> pins = process->get_terminalhatpins("DigitalInput",true);
			for(std::size_t i = 0; i < pins.size(); ++i)
			{
				std_msgs::Bool v;
				v.data = (bool)pins.at(i).Value;
				digitalinput_pubs.at(i).publish(v);
			}
		}
	}
	return true;
}
bool HatControllerNode::run_loop2()
{
	return true;
}
bool HatControllerNode::run_loop3()
{
	return true;
}

void HatControllerNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void HatControllerNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool HatControllerNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}
	else
	{
		if(process->is_ready() == false)
		{
			get_logger()->log_notice("Device not initialized yet.");
			eros::device::ConstPtr device_ptr(new eros::device(t_device));
			eros::diagnostic diag = process->new_devicemsg(device_ptr);

			if(diag.Level > INFO)
			{
				get_logger()->log_diagnostic(diag);
				diagnostic_pub.publish(diag);
			}
			if(process->is_ready() == true)
			{
				std::vector<HatControllerNodeProcess::Sensor> sensors = process->get_sensordata();
				for(std::size_t i = 0; i < sensors.size(); i++)
				{
					if(sensors.at(i).output_datatype == "signal")
					{
						std::string topic = "/" + sensors.at(i).name;
						ros::Publisher pub = n->advertise<eros::signal>(topic,10);
						signal_sensor_pubs.push_back(pub);
					}
				}
				{
					{//Terminal Hat
						std::vector<eros::pin> pins = process->get_terminalhatpins("",true);
						for(std::size_t i = 0; i < pins.size(); i++)
						{
							if(pins.at(i).Function == "DigitalInput")
							{
								std::string topic = "/" + pins.at(i).Name;
								ros::Publisher pub = n->advertise<std_msgs::Bool>(topic,10);
								signal_digitalinput_names.push_back(pins.at(i).Name);
								digitalinput_pubs.push_back(pub);
							}
							else if(pins.at(i).Function == "DigitalOutput")
							{
								std::string topic = "/" + pins.at(i).Name;
								ros::Subscriber sub = n->subscribe<eros::pin>(topic,5,&HatControllerNode::DigitalOutput_Callback,this);
								digitaloutput_subs.push_back(sub);
							}
							else
							{
								eros::diagnostic diagnostic;
								char tempstr[512];
								sprintf(tempstr,"[TerminalHat] Unsupported Pin Function: %s",pins.at(i).Function.c_str());
								diag = process->update_diagnostic(SOFTWARE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
								get_logger()->log_diagnostic(diag);
								kill_node = 1;
							}
						}
					}
					{//Servo Hats
						std::vector<uint16_t> addresses = process->get_servohataddresses();
						for(std::size_t i = 0; i < addresses.size(); ++i)
						{
							std::vector<eros::pin> pins = process->get_servohatpins(addresses.at(i));
							for(std::size_t j = 0; j < pins.size(); j++)
							{
								if((pins.at(i).Function == "PWMOutput") or (pins.at(i).Function == "PWMOutput-NonActuator"))
								{
									std::string topic = "/" + pins.at(j).Name;
									ros::Subscriber sub = n->subscribe<eros::pin>(topic,5,&HatControllerNode::PwmOutput_Callback,this);
									pwmoutput_subs.push_back(sub);
								}
							}
						}
					}
					{//GPIO Hats
						std::vector<uint16_t> addresses = process->get_gpiohataddresses();
						for(std::size_t i = 0; i < addresses.size(); ++i)
						{
							std::vector<eros::pin> pins = process->get_gpiohatpins(addresses.at(i));
							for(std::size_t j = 0; j < pins.size(); j++)
							{
								if(pins.at(i).Function == "DigitalOutput")
								{
									std::string topic = "/" + pins.at(i).Name;
									ros::Subscriber sub = n->subscribe<eros::pin>(topic,5,&HatControllerNode::DigitalOutput_Callback,this);
									digitaloutput_subs.push_back(sub);
								}
							}
						}
					}
				}
				get_logger()->log_notice("Device finished initializing.");
			}
			else
			{
			}
		}
		else
		{
		}
	}
	return true;
}
void HatControllerNode::ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	eros::diagnostic diag = process->new_armedstatemsg((uint8_t)msg->data);
	if(diag.Level > NOTICE)
	{
		diagnostic_pub.publish(diag);
		get_logger()->log_diagnostic(diag);

	}
}
void HatControllerNode::DigitalOutput_Callback(const eros::pin::ConstPtr& msg)
{
	double dt = ros::Time::now().toSec()-last_digitaloutput_time.toSec();
	//if(dt < .05) { return; } //Only update at 20 Hz
	last_digitaloutput_time = ros::Time::now();
	eros::diagnostic diagnostic = process->new_pinmsg(msg);
	if(diagnostic.Level > NOTICE)
	{
		get_logger()->log_diagnostic(diagnostic);
		diagnostic_pub.publish(diagnostic);
	}

}
void HatControllerNode::PwmOutput_Callback(const eros::pin::ConstPtr& msg)
{
	eros::diagnostic diagnostic = process->new_pinmsg(msg);
	if(diagnostic.Level > NOTICE)
	{
		get_logger()->log_diagnostic(diagnostic);
		diagnostic_pub.publish(diagnostic);
	}

}
void HatControllerNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void HatControllerNode::cleanup()
{
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	HatControllerNode *node = new HatControllerNode();
	bool status = node->start(argc,argv);
	std::thread thread(&HatControllerNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}
