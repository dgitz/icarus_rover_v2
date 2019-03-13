#include "audio_node.h"
bool kill_node = false;
bool AudioNode::start(int argc,char **argv)
{
	bool status = false;
	process = new AudioNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
	initialize_diagnostic(DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
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

	process->initialize(get_basenodename(),get_nodename(),get_hostname());
	process->set_diagnostic(diagnostic);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
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

eros::diagnostic AudioNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	std::string param_audiostage_dir = node_name + "/audiostage_directory";
	std::string audiostage_dir;
	if(n->getParam(param_audiostage_dir,audiostage_dir) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: audiostage_dir.  Exiting.";
		logger->log_diagnostic(diag);
		return diag;
	}
	if(process->set_audiostoragedirectory(audiostage_dir) == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Can't Set Audio Storage Dir: %s. Exiting.",audiostage_dir.c_str());
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = std::string(tempstr);
		logger->log_diagnostic(diag);
		return diag;
	}

	std::string param_volume = node_name + "/volume_perc";
	double volume;
	if(n->getParam(param_volume,volume) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: volume_perc.  Exiting.";
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_volume(volume);

	process->new_audioplaytrigger("Robot:Booting",true);
	process->enable_archive(false);

	std::string param_audiofile_length = node_name + "/audiofile_length";
	double audiofile_length;
	if(n->getParam(param_audiofile_length,audiofile_length) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: audiofile_length.  Exiting.";
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_audiorecord_duration(audiofile_length);

	std::string param_audiobuffer_length = node_name + "/audiobuffer_length";
	double audiobuffer_length;
	if(n->getParam(param_audiobuffer_length,audiobuffer_length) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: audiobuffer_length.  Exiting.";
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_totalaudiofiletimetokeep(audiobuffer_length);


	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic AudioNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&AudioNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&AudioNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,&AudioNode::ArmedState_Callback,this);
	return diagnostic;
}
bool AudioNode::run_001hz()
{
	return true;
}
bool AudioNode::run_01hz()
{
	eros::diagnostic diag = process->get_diagnostic();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool AudioNode::run_1hz()
{
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
		{
			eros::srv_device srv;
			srv.request.query = "DeviceType=Microphone";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
			}
		}
		{
			eros::srv_device srv;
			srv.request.query = "DeviceType=AudioAmplifier";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
			}
		}
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
	eros::diagnostic diag = process->get_diagnostic();
	if(diag.Level >= NOTICE)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}

	return true;
}
bool AudioNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool AudioNode::run_loop1()
{
	std::string command,filepath;
	if(process->get_audiorecordtrigger(command,filepath))
	{
		system(command.c_str());
	}
	return true;
}
bool AudioNode::run_loop2()
{
	return true;
}
bool AudioNode::run_loop3()
{
	return true;
}

void AudioNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void AudioNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool AudioNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}
	if((process->is_initialized() == true))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
void AudioNode::ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	process->new_armedstatemsg(msg->data);
}

void AudioNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void AudioNode::cleanup()
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
	AudioNode *node = new AudioNode();
	bool status = node->start(argc,argv);
	std::thread thread(&AudioNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

