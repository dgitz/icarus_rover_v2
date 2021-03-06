#include "audio_node.h"
bool kill_node = false;
bool AudioNode::start(int argc,char **argv)
{
	bool status = false;
	process = new AudioNodeProcess();
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
	diagnostic_types.push_back(SENSORS);
	diagnostic_types.push_back(REMOTE_CONTROL);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
	{
		diagnostic = process->update_diagnostic(SOFTWARE,INFO,INITIALIZING,"Node Configured.  Initializing.");
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic AudioNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
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

	std::string param_audiostage_dir = node_name + "/audiostage_directory";
	std::string audiostage_dir;
	if(n->getParam(param_audiostage_dir,audiostage_dir) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: audiostage_dir.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	if(process->set_audiostoragedirectory(audiostage_dir) == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Can't Set Audio Storage Dir: %s. Exiting.",audiostage_dir.c_str());
		diag.Description = std::string(tempstr);
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
		return diag;
	}

	std::string param_volume = node_name + "/volume_perc";
	double volume;
	if(n->getParam(param_volume,volume) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR, "Missing Parameter: volume_perc.  Exiting.");
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
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: audiofile_length.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_audiorecord_duration(audiofile_length);

	std::string param_audiobuffer_length = node_name + "/audiobuffer_length";
	double audiobuffer_length;
	if(n->getParam(param_audiobuffer_length,audiobuffer_length) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: audiobuffer_length.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_totalaudiofiletimetokeep(audiobuffer_length);

	diag = process->update_diagnostic(DATA_STORAGE,NOTICE,INITIALIZING,"Configuration Files Loaded.");
	logger->log_diagnostic(diag);
	return diagnostic;
}
bool AudioNode::run_001hz()
{
	return true;
}
bool AudioNode::run_01hz()
{
	return true;
}
bool AudioNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool AudioNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
	}
	else if((process->get_taskstate() == TASKSTATE_INITIALIZED) and (process->get_query_for_device_configuration()))
	{
		{
			eros::srv_device srv;
			srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_MICROPHONE)).c_str();
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
				if(srv.response.data.size() == 0)
				{
					process->update_diagnostic(SENSORS,WARN,DEVICE_NOT_AVAILABLE,"No Microphone Available.");
				}
			}
		}
		{
			eros::srv_device srv;
			srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_AUDIOAMPLIFIER)).c_str();
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
				if(srv.response.data.size() == 0)
				{
					process->update_diagnostic(REMOTE_CONTROL,WARN,DEVICE_NOT_AVAILABLE,"No Audio Amplifier Available.");
				}
			}
		}
		process->set_query_for_device_configuration(false);
	}
	else if (process->get_taskstate() == TASKSTATE_INITIALIZING)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{

					get_logger()->log_error(__FILE__,__LINE__,"Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query,srv.response.data.at(0));
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
bool AudioNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
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
	if (process->get_taskstate() == TASKSTATE_INITIALIZED)
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
	base_cleanup();
	get_logger()->log_info(__FILE__,__LINE__,"[AudioNode] Finished Safely.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing AudioNode with Signal: %d", sig);
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
		status = node->update(node->get_process()->get_taskstate());
	}
	node->cleanup();
	thread.detach();
	return 0;
}