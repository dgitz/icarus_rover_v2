#include "hatcontroller_node.h"
//Start User Code: Firmware Definition
#define HATCONTROLLERNODE_MAJOR_RELEASE 1
#define HATCONTROLLERNODE_MINOR_RELEASE 1
#define HATCONTROLLERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	icarus_rover_v2::diagnostic diagnostic = process->update(1.0/(double)loop1_rate);
	bool process_ready = process->is_ready();
	if(process_ready == true)
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
					diagnostic.Diagnostic_Type = COMMUNICATIONS;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
					char tempstr[512];
					sprintf(tempstr,"Unable to Start ServoHat at Address: %d",ServoHats_ids.at(i));
					diagnostic.Description = std::string(tempstr);
					process->set_diagnostic(diagnostic);
					diagnostic_pub.publish(diagnostic);
					logger->log_diagnostic(diagnostic);
					printf("[%s]: %s\n",node_name.c_str(),tempstr);
					return false;
				}
				diagnostic = process->set_hat_running("ServoHat",ServoHats_ids.at(i));
				logger->log_diagnostic(diagnostic);
				if(diagnostic.Level > INFO) { diagnostic_pub.publish(diagnostic); }
			}
			else
			{
				if(ServoHats_ids.at(i) == ServoHats.at(i).get_address())
				{
					std::vector<icarus_rover_v2::pin> pins = process->get_servohatpins(ServoHats_ids.at(i));
					for(std::size_t j = 0; j < pins.size(); j++)
					{
						ServoHats.at(i).setServoValue(pins.at(j).Number, pins.at(j).Value);
					}
				}
				else
				{
					diagnostic.Diagnostic_Type = SOFTWARE;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
					char tempstr[512];
					sprintf(tempstr,"Trying to Set ServoHat Address: %d but got: %d",ServoHats_ids.at(i),ServoHats.at(i).get_address());
					diagnostic.Description = std::string(tempstr);
					logger->log_diagnostic(diagnostic);
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
					diagnostic.Diagnostic_Type = COMMUNICATIONS;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
					char tempstr[512];
					sprintf(tempstr,"Unable to Start GPIOHat at Address: %d",GPIOHats_ids.at(i));
					diagnostic.Description = std::string(tempstr);
					process->set_diagnostic(diagnostic);
					diagnostic_pub.publish(diagnostic);
					logger->log_diagnostic(diagnostic);
					printf("[%s]: %s\n",node_name.c_str(),tempstr);
					return false;
				}
				diagnostic = process->set_hat_running("GPIOHat",GPIOHats_ids.at(i));

				logger->log_diagnostic(diagnostic);
				if(diagnostic.Level > INFO) { diagnostic_pub.publish(diagnostic); }
			}
			else
			{
				if(GPIOHats_ids.at(i) == GPIOHats.at(i).get_address())
				{
					std::vector<icarus_rover_v2::pin> pins = process->get_gpiohatpins(GPIOHats_ids.at(i));

				}
				else
				{
					diagnostic.Diagnostic_Type = SOFTWARE;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
					char tempstr[512];
					sprintf(tempstr,"Trying to Set GPIOHat Address: %d but got: %d",GPIOHats_ids.at(i),GPIOHats.at(i).get_address());
					diagnostic.Description = std::string(tempstr);
					logger->log_diagnostic(diagnostic);
				}
			}

		}
		if(process->is_hat_running("TerminalHat",0) == false)
		{
			TerminalHat.init();
			{
				bool any_error = false;
				std::vector<icarus_rover_v2::pin> pins = process->get_terminalhatpins("");
				for(std::size_t i = 0; i < pins.size(); i++)
				{
					if(TerminalHat.configure_pin(pins.at(i).Number,pins.at(i).Function) == false)
					{
						any_error = true;
						diagnostic.Diagnostic_Type = SOFTWARE;
						diagnostic.Level = ERROR;
						diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
						char tempstr[512];
						sprintf(tempstr,"[TerminalHat] Could not configure Pin: %d with Function: %s",pins.at(i).Number,pins.at(i).Function.c_str());
						diagnostic.Description = std::string(tempstr);
						process->set_diagnostic(diagnostic);
						printf("%s\n",tempstr);
						logger->log_error(std::string(tempstr));
						kill_node = 1;
					}
				}
				if(any_error == false)
				{
					diagnostic = process->set_hat_running("TerminalHat",0);
					logger->log_diagnostic(diagnostic);
					if(diagnostic.Level > INFO) { diagnostic_pub.publish(diagnostic); }
				}
			}

		}
	}
	if(diagnostic.Level > INFO)
	{
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
	icarus_rover_v2::diagnostic diag;
	for(std::size_t i = 0; i < GPIOHats.size(); i++)
	{
		{

			if(GPIOHats.at(i).get_lock() == false)
			{
				GPIOHats.at(i).lockdevice();
				unsigned char inputbuffer[13];
				int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_DIO_Port1_ID,inputbuffer);
				if(passed_checksum_calc > 0)
				{
					uint16_t a1,a2,a3,a4;
					int length;
					bool success = i2cmessagehandler->decode_Get_DIO_Port1I2C(inputbuffer,&length,&a1,&a2,&a3,&a4);
					if(success == true)
					{
						diag = process->new_message_GetDIOPort1(GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4);
						if(diag.Level >= WARN)
						{
							diagnostic_pub.publish(diag);
							logger->log_diagnostic(diag);
						}
						std::vector<HatControllerNodeProcess::Sensor> sensors = process->get_sensordata();
						for(std::size_t i = 0; i < sensors.size(); i++)
						{
							for(std::size_t j = 0; j < signal_sensor_names.size(); j++)
							{
								if(signal_sensor_names.at(j) == sensors.at(i).name)
								{
									icarus_rover_v2::signal v;
									v.tov = ros::Time(sensors.at(i).tov);
									v.value = sensors.at(i).value;
									v.units = sensors.at(i).units;
									v.status = sensors.at(i).status;
									v.rms = -1.0;
									signal_sensor_pubs.at(j).publish(v);
								}
							}
						}
					}
				}
			}
		}
	}

	{
		std::vector<icarus_rover_v2::pin> pins = process->get_terminalhatpins("DigitalInput");
		for(std::size_t i = 0; i < pins.size(); i++)
		{
			if(pins.at(i).Name == digitalinput_names.at(i))
			{
				std_msgs::Bool v;
				v.data = (bool)pins.at(i).Value;
				digitalinput_pubs.at(i).publish(v);
			}
		}
	}
	bool ready_to_arm = process->get_ready_to_arm();
	std_msgs::Bool bool_ready_to_arm;
	bool_ready_to_arm.data = ready_to_arm;
	ready_to_arm_pub.publish(bool_ready_to_arm);
	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{
	icarus_rover_v2::diagnostic diag;
	if(process->is_ready() == true)
	{
		for(std::size_t i = 0; i < GPIOHats.size(); i++)
		{
			if(GPIOHats.at(i).get_lock() == false)
			{
				GPIOHats.at(i).lockdevice();
				unsigned char inputbuffer[13];
				int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_IMUAcc_ID,inputbuffer);
				if(passed_checksum_calc > 0)
				{
					uint16_t a1,a2,a3,a4,a5,a6;
					int length;
					bool success = i2cmessagehandler->decode_Get_IMUAccI2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
					if(success == true)
					{
						diag = process->new_message_IMUAcc(GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
						if(diag.Level >= WARN)
						{
							diagnostic_pub.publish(diag);
							logger->log_diagnostic(diag);
						}
					}
				}
			}

			if(GPIOHats.at(i).get_lock() == false)
			{
				GPIOHats.at(i).lockdevice();
				unsigned char inputbuffer[13];
				int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_IMUGyro_ID,inputbuffer);
				if(passed_checksum_calc > 0)
				{
					uint16_t a1,a2,a3,a4,a5,a6;
					int length;
					bool success = i2cmessagehandler->decode_Get_IMUGyroI2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
					if(success == true)
					{
						diag = process->new_message_IMUGyro(GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
						if(diag.Level >= WARN)
						{
							diagnostic_pub.publish(diag);
							logger->log_diagnostic(diag);
						}
					}
				}
			}
			if(GPIOHats.at(i).get_lock() == false)
			{
				GPIOHats.at(i).lockdevice();
				unsigned char inputbuffer[13];
				int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Get_IMUMag_ID,inputbuffer);
				if(passed_checksum_calc > 0)
				{
					uint16_t a1,a2,a3,a4,a5,a6;
					int length;
					bool success = i2cmessagehandler->decode_Get_IMUMagI2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
					if(success == true)
					{
						diag = process->new_message_IMUMag(GPIOHats.at(i).get_address(),ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
						if(diag.Level >= WARN)
						{
							diagnostic_pub.publish(diag);
							logger->log_diagnostic(diag);
						}
					}
				}
			}
			icarus_rover_v2::imu imu1,imu2;
			int v1 = process->get_imu(&imu1,1);
			if(v1 > 0)
			{
				imu1raw_pub.publish(imu1);
			}
			else if(v1 == 0)
			{
			}
			int v2 = process->get_imu(&imu2,2);
			if(v2 > 0)
			{
				imu2raw_pub.publish(imu2);
			}
		}
	}

	return true;
}
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	icarus_rover_v2::diagnostic diagnostic = process->new_armedstatemsg((uint8_t)msg->data);
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
	}
}
/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "hatcontroller_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 22-August-2018";
	fw.Major_Release = HATCONTROLLERNODE_MAJOR_RELEASE;
	fw.Minor_Release = HATCONTROLLERNODE_MINOR_RELEASE;
	fw.Build_Number = HATCONTROLLERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	printf("t=%4.2f (sec) [%s]: %s\n",ros::Time::now().toSec(),node_name.c_str(),process->get_diagnostic().Description.c_str());
}
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	double dt = measure_time_diff(ros::Time::now(),last_digitaloutput_time);
	//if(dt < .05) { return; } //Only update at 20 Hz
	last_digitaloutput_time = ros::Time::now();
	icarus_rover_v2::pin pinmsg;
	pinmsg.ParentDevice = msg->ParentDevice;
	pinmsg.Number = msg->Number;
	pinmsg.Function = msg->Function;
	pinmsg.DefaultValue = msg->DefaultValue;
	pinmsg.ConnectedDevice = msg->ConnectedDevice;
	pinmsg.Value = msg->Value;
	icarus_rover_v2::diagnostic diagnostic = process->new_pinmsg(pinmsg);
	if(diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
		diagnostic_pub.publish(diagnostic);
	}

}
void PwmOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	//ros::Time now = ros::Time::now();

	//double dt = measure_time_diff(now,last_pwmoutput_sub_time);
	//if(dt < .05) { return; } //Only update at 20 Hz
	//last_pwmoutput_sub_time = ros::Time::now();

	icarus_rover_v2::pin pinmsg;
	pinmsg.stamp = msg->stamp;
	pinmsg.ParentDevice = msg->ParentDevice;
	pinmsg.Number = msg->Number;
	pinmsg.Function = msg->Function;
	pinmsg.DefaultValue = msg->DefaultValue;
	pinmsg.ConnectedDevice = msg->ConnectedDevice;
	pinmsg.Value = msg->Value;
	icarus_rover_v2::diagnostic diagnostic = process->new_pinmsg(pinmsg);
	if(diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
		diagnostic_pub.publish(diagnostic);
	}

}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	if(process->get_analyzetiming() == true)
	{
		char tempstr[512];
		sprintf(tempstr,"Average Time delay for pin update: %f (sec)",process->get_timedelay());
		logger->log_info(std::string(tempstr));
	}
	received_pps = true;
	std_msgs::Bool pps;
	pps.data = msg->data;
	icarus_rover_v2::diagnostic diagnostic = process->new_ppsmsg(pps);
	if(diagnostic.Level > NOTICE) {   diagnostic_pub.publish(diagnostic); }
	if((process->get_initialized() == true) and (process->get_ready() == true))
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_warn("Couldn't read resources used.");
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
	else if((process->get_ready() == false) and (process->get_initialized() == true))
	{
		{
			icarus_rover_v2::srv_device srv;
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
	else if(process->get_initialized() == false)
	{
		{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{
					logger->log_error("Got unexpected device message");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
		}
	}
	else
	{
	}
	/*
	for(std::size_t i = 0; i < GPIOHats.size(); i++)
	{
		unsigned char inputbuffer[12];
		int length;
		unsigned char System,Subsystem,Component,DiagType,Level,Message;
		int passed_checksum_calc = GPIOHats.at(i).sendQuery(I2CMessageHandler::I2C_Diagnostic_ID,inputbuffer);
		if(passed_checksum_calc > 0)
		{
			bool success = i2cmessagehandler->decode_DiagnosticI2C(inputbuffer,&length,&System,&Subsystem,&Component,&DiagType,&Level,&Message);
			if(success == true)
			{
				/*diag = process->new_message_Diag(GPIOHats.at(i).get_address(),ros::Time::now().toSec(),System,Subsystem,Component,DiagType,Level,Message);
				if(diag.Level >= WARN)
				{
					diagnostic_pub.publish(diag);
					logger->log_diagnostic(diag);
				}
	 */
	diagnostic_pub.publish(process->get_diagnostic());
}
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::command command;
	command.Command = msg->Command;
	command.Option1 = msg->Option1;
	command.Option2 = msg->Option2;
	command.Option3 = msg->Option3;
	command.CommandText = msg->CommandText;
	command.Description = msg->Description;
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(command);
	for(std::size_t i = 0; i < diaglist.size(); i++)
	{
		if(diaglist.at(i).Level >= NOTICE)
		{
			logger->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
		if((diaglist.at(i).Level > NOTICE) and (process->get_runtime() > 10.0))
		{
			printf("[%s]: %s\n",node_name.c_str(),diaglist.at(i).Description.c_str());
		}
	}
}
//End User Code: Functions
bool run_10Hz_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	if(process->get_diagnostic().Level > NOTICE)
	{
		diagnostic_pub.publish(process->get_diagnostic());
		logger->log_diagnostic(process->get_diagnostic());
	}
	return true;
}
int main(int argc, char **argv)
{
	node_name = "hatcontroller_node";
	ros::init(argc, argv, node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	ros::NodeHandle n;

	if(initializenode() == false)
	{
		char tempstr[256];
		sprintf(tempstr,"Unable to Initialize. Exiting.");
		printf("[%s]: %s\n",node_name.c_str(),tempstr);
		logger->log_fatal(tempstr);
		kill_node = 1;
	}
	ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
	last_10Hz_timer = ros::Time::now();
	double mtime;
	while (ros::ok() && (kill_node == 0))
	{
		bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
		if(ok_to_start == true)
		{
			if(run_loop1 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
				if(mtime >= (1.0/loop1_rate))
				{
					run_loop1_code();
					last_loop1_timer = ros::Time::now();
				}
			}
			if(run_loop2 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
				if(mtime >= (1.0/loop2_rate))
				{
					run_loop2_code();
					last_loop2_timer = ros::Time::now();
				}
			}
			if(run_loop3 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
				if(mtime >= (1.0/loop3_rate))
				{
					run_loop3_code();
					last_loop3_timer = ros::Time::now();
				}
			}

			mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
			if(mtime >= 0.1)
			{
				run_10Hz_code();
				last_10Hz_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	kill_node = true;
	sleep(2.0);
	logger->log_notice("Node Finished Safely.");
	return 0;
}
bool initializenode()
{
	//Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,5);
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = hostname;
	diagnostic.Node_Name = node_name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = GPIO_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,5);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("WARN",ros::this_node::getName());
		logger->log_warn("Missing Parameter: verbosity_level");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}
	std::string param_disabled = node_name +"/disable";
	bool disable_node;
	if(n->getParam(param_disabled,disable_node) == true)
	{
		if(disable_node == true)
		{
			logger->log_notice("Node Disabled in Launch File.  Exiting.");
			printf("[%s]: Node Disabled in Launch File. Exiting.\n",node_name.c_str());
			return false;
		}
	}

	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,5);
	beat.Node_Name = node_name;

	std::string param_startup_delay = node_name + "/startup_delay";
	double startup_delay = 0.0;
	if(n->getParam(param_startup_delay,startup_delay) == false)
	{
		logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
	}
	else
	{
		char tempstr[128];
		sprintf(tempstr,"Using Parameter: startup_delay = %4.2f sec.",startup_delay);
		logger->log_notice(std::string(tempstr));
	}
	printf("[%s] Using Parameter: startup_delay = %4.2f sec.\n",node_name.c_str(),startup_delay);
	ros::Duration(startup_delay).sleep();

	std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);

	pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",5,PPS01_Callback);
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",5,PPS1_Callback);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",5,Command_Callback);
	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
	firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1);

	double max_rate = 0.0;
	std::string param_loop1_rate = node_name + "/loop1_rate";
	if(n->getParam(param_loop1_rate,loop1_rate) == false)
	{
		logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
		run_loop1 = false;
	}
	else
	{
		last_loop1_timer = ros::Time::now();
		run_loop1 = true;
		if(loop1_rate > max_rate) { max_rate = loop1_rate; }
	}

	std::string param_loop2_rate = node_name + "/loop2_rate";
	if(n->getParam(param_loop2_rate,loop2_rate) == false)
	{
		logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
		run_loop2 = false;
	}
	else
	{
		last_loop2_timer = ros::Time::now();
		run_loop2 = true;
		if(loop2_rate > max_rate) { max_rate = loop2_rate; }
	}

	std::string param_loop3_rate = node_name + "/loop3_rate";
	if(n->getParam(param_loop3_rate,loop3_rate) == false)
	{
		logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
		run_loop3 = false;
	}
	else
	{
		last_loop3_timer = ros::Time::now();
		run_loop3 = true;
		if(loop3_rate > max_rate) { max_rate = loop3_rate; }
	}
	ros_rate = max_rate * 50.0;
	if(ros_rate < 100.0) { ros_rate = 100.0; }
	char tempstr[512];
	sprintf(tempstr,"Running Node at Rate: %f",ros_rate);
	logger->log_notice(std::string(tempstr));
	//End Template Code: Initialization and Parameters

	//Start User Code: Initialization and Parameters
	process = new HatControllerNodeProcess;
	std::string sensor_spec_path;
	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,ArmedState_Callback);

	bool analyze_timing;
	process->init(diagnostic,hostname);
	if(diagnostic.Level > NOTICE)
	{
		logger->log_fatal(diagnostic.Description);
		printf("[%s]: %s\n",node_name.c_str(),diagnostic.Description.c_str());
		return false;
	}
	std::string param_analyze_timing = node_name + "/analyze_timing";
	if(n->getParam(param_analyze_timing,analyze_timing) == false)
	{
		logger->log_notice("Missing parameter: analyze_timing.  Not analyzing timing.");
	}
	else
	{
		if(analyze_timing == true)
		{
			logger->log_notice("Analyzing Timing.");
		}
		process->set_analyzetiming(analyze_timing);
	}
	last_message_received_time = ros::Time::now();
	std::string digitalinput_topic = "/" + node_name + "/DigitalInput";
	digitalinput_pub = n->advertise<icarus_rover_v2::pin>(digitalinput_topic,1);
	std::string analoginput_topic = "/" + node_name + "/AnalogInput";
	analoginput_pub = n->advertise<icarus_rover_v2::pin>(analoginput_topic,1);
	std::string forcesensorinput_topic = "/" + node_name + "/ForceSensorInput";
	forcesensorinput_pub = n->advertise<icarus_rover_v2::pin>(forcesensorinput_topic,1);
	std::string digitaloutput_topic = "/" + node_name + "/DigitalOutput";
	digitaloutput_sub = n->subscribe<icarus_rover_v2::pin>(digitaloutput_topic,5,DigitalOutput_Callback);
	last_digitaloutput_time = ros::Time::now();
	std::string pwmoutput_topic = "/" + node_name + "/PWMOutput";
	pwmoutput_sub = n->subscribe<icarus_rover_v2::pin>(pwmoutput_topic,5,PwmOutput_Callback);
	last_pwmoutput_sub_time = ros::Time::now();
	std::string imu1raw_topic = "/IMU1_raw";
	imu1raw_pub = n->advertise<icarus_rover_v2::imu>(imu1raw_topic,1);
	std::string imu2raw_topic = "/IMU2_raw";
	imu2raw_pub = n->advertise<icarus_rover_v2::imu>(imu2raw_topic,1);

	std::string ready_to_arm_topic = node_name + "/ready_to_arm";
	ready_to_arm_pub = n->advertise<std_msgs::Bool>(ready_to_arm_topic,1);

	ServoHats.clear();
	GPIOHats.clear();


	//Finish User Code: Initialization and Parameters

	//Start Template Code: Final Initialization.
	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Initialized";
	process->set_diagnostic(diagnostic);
	diagnostic_pub.publish(diagnostic);
	logger->log_info("Initialized!");
	return true;
	//End Template Code: Finish Initialization.
}
//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{
	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{
			process->set_mydevice(device);
			myDevice = process->get_mydevice();
			resourcemonitor = new ResourceMonitor(process->get_diagnostic(),device.Architecture,device.DeviceName,node_name);

		}
	}
	else
	{
		if(process->is_ready() == false)
		{
			logger->log_notice("Device not initialized yet.");

			icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
			logger->log_diagnostic(diag);
			if(diag.Level > INFO) { diagnostic_pub.publish(diag); }
			if(process->is_ready() == true)
			{
				std::vector<HatControllerNodeProcess::Sensor> sensors = process->get_sensordata();
				for(std::size_t i = 0; i < sensors.size(); i++)
				{
					if(sensors.at(i).output_datatype == "signal")
					{
						std::string topic = "/" + sensors.at(i).name;
						ros::Publisher pub = n->advertise<icarus_rover_v2::signal>(topic,10);
						signal_sensor_pubs.push_back(pub);
						signal_sensor_names.push_back(sensors.at(i).name);
					}
				}
				{
					std::vector<icarus_rover_v2::pin> pins = process->get_terminalhatpins("");
					for(std::size_t i = 0; i < pins.size(); i++)
					{
						if(pins.at(i).Function == "DigitalInput")
						{
							std::string topic = "/" + pins.at(i).Name;
							ros::Publisher pub = n->advertise<std_msgs::Bool>(topic,10);
							digitalinput_names.push_back(pins.at(i).Name);
							digitalinput_pubs.push_back(pub);
						}
						else
						{
							icarus_rover_v2::diagnostic diagnostic;
							diagnostic.Diagnostic_Type = SOFTWARE;
							diagnostic.Level = ERROR;
							diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
							char tempstr[512];
							sprintf(tempstr,"[TerminalHat] Unsupported Pin Function: %s",pins.at(i).Function.c_str());
							diagnostic.Description = std::string(tempstr);
							process->set_diagnostic(diagnostic);
							printf("%s\n",tempstr);
							logger->log_error(std::string(tempstr));
							kill_node = 1;
						}
					}
				}
				logger->log_notice("Device finished initializing.");
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
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
