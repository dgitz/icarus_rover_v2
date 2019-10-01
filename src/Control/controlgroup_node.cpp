#include "controlgroup_node.h"
bool kill_node = false;
bool ControlGroupNode::start(int argc, char **argv)
{
	bool status = false;
	process = new ControlGroupNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
	diagnostic = preinitialize_basenode(argc, argv);
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if (diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(), get_nodename(), get_hostname(), DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
	process->set_config_filepaths("/home/robot/config/ControlGroup.xml");
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	if (diagnostic.Level < WARN)
	{
		diagnostic.Diagnostic_Type = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Node Configured.  Initializing.";
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic ControlGroupNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic ControlGroupNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS", 1, &ControlGroupNode::PPS1_Callback, this);
	command_sub = n->subscribe<eros::command>("/command", 1, &ControlGroupNode::Command_Callback, this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string pin_topic = "/" + std::string(host_name) + "_master_node/srv_pin";
	srv_pin = n->serviceClient<eros::srv_pin>(pin_topic);
	std::string tune_controlgroup_topic = "/DriverStation/tune_controlgroup";
	tune_controlgroup_sub = n->subscribe<eros::tune_controlgroup>(tune_controlgroup_topic,1,&ControlGroupNode::TuneControlGroup_Callback,this);
	//Initialize Output Publishers
	std::vector<eros::pin> pins = process->get_outputpins();
	for(std::size_t i = 0; i < pins.size(); ++i)
	{
		printf("Init Publisher: %s\n",pins.at(i).ConnectedDevice.c_str());
		ros::Publisher pub = n->advertise<eros::pin>("/" + pins.at(i).Name,1);
		outputs.push_back(pub);
	}
	//Initialize Input Subscribers
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{
		std::vector<std::string> input_signals = controlgroups.at(i).get_inputsignalnames();
		for(std::size_t j = 0; j < input_signals.size(); ++j)
		{
			printf("Init Subscriber: %s\n",input_signals.at(j).c_str());
			ros::Subscriber sub = n->subscribe<eros::signal>("/" + input_signals.at(j),1,&ControlGroupNode::Signal_Callback,this);
			inputs.push_back(sub);
		}
	}
	return diagnostic;
}
bool ControlGroupNode::run_001hz()
{
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); ++i)
	{

	}
	return true;
}
bool ControlGroupNode::run_01hz()
{

	return true;
}
bool ControlGroupNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool ControlGroupNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if ((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if ((process->is_ready() == false) and (process->is_initialized() == true))
	{
		std::vector<eros::pin> outputs = process->get_outputpins();
		for(std::size_t i = 0; i < outputs.size(); ++i)
		{
			eros::srv_pin srv;
			srv.request.query = outputs.at(i).ConnectedDevice;
			if (srv_pin.call(srv) == true)
			{
				if (srv.response.pins.size() != 1)
				{
					get_logger()->log_error("Got unexpected pin message.");
				}
				else
				{
					process->set_pinproperties(srv.response.pins.at(0));
				}
			}
		}
		

	}
	else if (process->is_initialized() == false)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if (srv_device.call(srv) == true)
			{
				if (srv.response.data.size() != 1)
				{

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query, srv.response.data.at(0));
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
bool ControlGroupNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1, ros::Time::now().toSec());
	if (diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
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
bool ControlGroupNode::run_loop1()
{
	std::vector<eros::pin> pins = process->get_outputpins();
	for(std::size_t i = 0; i < pins.size(); ++i)
	{
		outputs.at(i).publish(pins.at(i));
	}
	return true;
}
bool ControlGroupNode::run_loop2()
{
	return true;
}
bool ControlGroupNode::run_loop3()
{
	return true;
}
void ControlGroupNode::TuneControlGroup_Callback(const eros::tune_controlgroup::ConstPtr& t_msg)
{
	process->set_PIDGains(t_msg->Name,t_msg->param1,t_msg->param2,t_msg->param3);
}
void ControlGroupNode::PPS1_Callback(const std_msgs::Bool::ConstPtr &msg)
{
	new_ppsmsg(msg);
}
void ControlGroupNode::Signal_Callback(const eros::signal::ConstPtr& t_msg)
{
	process->new_inputsignalmsg(t_msg);

}
void ControlGroupNode::Command_Callback(const eros::command::ConstPtr &t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg, diaglist);
}
bool ControlGroupNode::new_devicemsg(std::string query, eros::device t_device)
{
	if (query == "SELF")
	{
		if (t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}

	if ((process->is_initialized() == true))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
void ControlGroupNode::thread_loop()
{
	while (kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void ControlGroupNode::cleanup()
{
	base_cleanup();
	get_logger()->log_info("Node Finished Safely.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing Node with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv)
{
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	ControlGroupNode *node = new ControlGroupNode();
	bool status = node->start(argc, argv);
	std::thread thread(&ControlGroupNode::thread_loop, node);
	while ((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	thread.detach();
	return 0;
}
