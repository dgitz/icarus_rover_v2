#include "safety_node.h"
//Start User Code: Firmware Definition
#define SAFETYNODE_MAJOR_RELEASE 1
#define SAFETYNODE_MINOR_RELEASE 2
#define SAFETYNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	icarus_rover_v2::diagnostic diag = process->update(1.0/(double)loop1_rate);
	if(diag.Level >= NOTICE)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
	if(process->is_hat_running("TerminalHat",0) == true)
	{
		int pin_value = TerminalHat.read_pin(process->get_pinnumber("ArmSwitch"));
		bool v = process->set_pinvalue("ArmSwitch",pin_value);
		if(v == false)
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			char tempstr[512];
			sprintf(tempstr,"[TerminalHat] Could not read Arm Switch");
			diag.Description = std::string(tempstr);
			process->set_diagnostic(diag);
			logger->log_error(std::string(tempstr));
			kill_node = 1;
		}
	}
	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
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
	return true;
}
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	//diagnostic_status = process->new_armedstatemsg(msg->data);
	icarus_rover_v2::diagnostic diagnostic = process->get_diagnostic();
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
}
/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = process->get_basenodename();
	fw.Node_Name = process->get_nodename();
	fw.Description = "Latest Rev: 26-October-2018";
	fw.Major_Release = SAFETYNODE_MAJOR_RELEASE;
	fw.Minor_Release = SAFETYNODE_MINOR_RELEASE;
	fw.Build_Number = SAFETYNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	logger->log_diagnostic(process->get_diagnostic());
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	received_pps = true;
	if((process->get_initialized() == true) and (process->get_ready() == true))
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_diagnostic(resource_diagnostic);
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
		icarus_rover_v2::srv_device srv;
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
						diag.Diagnostic_Type = SOFTWARE;
						diag.Level = ERROR;
						diag.Diagnostic_Message = INITIALIZING_ERROR;
						char tempstr[512];
						sprintf(tempstr,"[TerminalHat] Could not configure Pin: %d with Function: %s",pins.at(i).Number,pins.at(i).Function.c_str());
						diag.Description = std::string(tempstr);
						process->set_diagnostic(diag);
						logger->log_error(std::string(tempstr));
						kill_node = 1;
					}
				}
				if(any_error == false)
				{
					diag = process->set_terminalhat_initialized();
					logger->log_diagnostic(diag);
					if(diag.Level > NOTICE) { diagnostic_pub.publish(diag); }
				}
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
	if((command.Option1 >= LEVEL3) and (diaglist.size() == 1) and (diaglist.at(0).Diagnostic_Message == DIAGNOSTIC_PASSED))
	{
		logger->log_diagnostic(diaglist.at(0));
		diagnostic_pub.publish(diaglist.at(0));

	}
	else
	{
		for(std::size_t i = 0; i < diaglist.size(); i++)
		{
			if(diaglist.at(i).Level > NOTICE)
			{
				logger->log_diagnostic(diaglist.at(i));
				diagnostic_pub.publish(diaglist.at(i));
			}
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
	base_node_name = "safety_node";
	ros::init(argc, argv, base_node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	ros::NodeHandle n;

	if(initializenode() == false)
	{
		char tempstr[256];
		sprintf(tempstr,"Unable to Initialize. Exiting.");
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
	diagnostic.Component = DIAGNOSTIC_NODE;

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
	if(ros_rate > 100.0) { ros_rate = 100.0; }
	char tempstr[512];
	sprintf(tempstr,"Running Node at Rate: %4.2f",ros_rate);
	logger->log_notice(std::string(tempstr));
	//End Template Code: Initialization and Parameters

	//Start User Code: Initialization and Parameters
	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,ArmedState_Callback);

	process = new SafetyNodeProcess(base_node_name,node_name);
	diagnostic = process->init(diagnostic,std::string(hostname));
	if(diagnostic.Level > NOTICE)
	{
		logger->log_fatal(diagnostic.Description);
		printf("[%s]: %s\n",node_name.c_str(),diagnostic.Description.c_str());
		return false;
	}
	std::string ready_to_arm_topic = node_name + "/ready_to_arm";
	ready_to_arm_pub = n->advertise<std_msgs::Bool>(ready_to_arm_topic,1);

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
			resourcemonitor = new ResourceMonitor(process->get_diagnostic(),device.Architecture,device.DeviceName,node_name);
			process->set_mydevice(device);
		}
	}

	if((process->get_initialized() == true))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
	}
	return true;
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
