#include "audio_node.h"
//Start User Code: Firmware Definition
#define AUDIONODE_MAJOR_RELEASE 0
#define AUDIONODE_MINOR_RELEASE 0
#define AUDIONODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	std::string command,filepath;
	if(process->get_audiorecordtrigger(command,filepath))
	{
		system(command.c_str());
	}
	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
	icarus_rover_v2::diagnostic diag = process->update(ros::Time::now().toSec(),measure_time_diff(ros::Time::now(),last_loop2_timer));
	if(diag.Level >= WARN)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{
	return true;
}
/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "audio_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 17-September-2018";
	fw.Major_Release = AUDIONODE_MAJOR_RELEASE;
	fw.Minor_Release = AUDIONODE_MINOR_RELEASE;
	fw.Build_Number = AUDIONODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	printf("t=%4.2f (sec) [%s]: %s\n",ros::Time::now().toSec(),node_name.c_str(),process->get_diagnostic().Description.c_str());
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
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
    	    		icarus_rover_v2::srv_device srv;
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
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	uint8_t armed_state = msg->data;

	//diagnostic_status = process->new_armedstatemsg(msg->data);
	process->new_armedstatemsg(armed_state);
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
	node_name = "audio_node";
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
	diagnostic.Component = TIMING_NODE;

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
    process = new AudioNodeProcess;
    
	diagnostic = process->init(diagnostic,std::string(hostname));
	if(diagnostic.Level > NOTICE)
	{
		logger->log_fatal(diagnostic.Description);
		printf("[%s]: %s\n",node_name.c_str(),diagnostic.Description.c_str());
		return false;
	}
	std::string param_audiostage_dir = node_name + "/audiostage_directory";
	std::string audiostage_dir;
	if(n->getParam(param_audiostage_dir,audiostage_dir) == false)
	{
		logger->log_error("Missing parameter: audiostage_dir.  Exiting.");
		return false;
	}
	if(process->set_audiostoragedirectory(audiostage_dir) == false)
	{
		printf("Can't Set: %s. Exiting.\n",audiostage_dir.c_str());
	}

	std::string param_volume = node_name + "/volume_perc";
	double volume;
	if(n->getParam(param_volume,volume) == false)
	{
		logger->log_error("Missing parameter: volume_perc.  Exiting.");
		return false;
	}
	process->set_volume(volume);

	process->new_audioplaytrigger("Robot:Booting",true);
	process->enable_archive(false);

	std::string param_audiofile_length = node_name + "/audiofile_length";
	double audiofile_length;
	if(n->getParam(param_audiofile_length,audiofile_length) == false)
	{
		logger->log_error("Missing parameter: audiofile_length.  Exiting.");
		return false;
	}
	process->set_audiorecord_duration(audiofile_length);

	std::string param_audiobuffer_length = node_name + "/audiobuffer_length";
	double audiobuffer_length;
	if(n->getParam(param_audiobuffer_length,audiobuffer_length) == false)
	{
		logger->log_error("Missing parameter: audiobuffer_length.  Exiting.");
		return false;
	}
	process->set_totalaudiofiletimetokeep(audiobuffer_length);
	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,ArmedState_Callback);


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
	exit(0);
	system("sudo killall network_transceiver_node");
	kill_node = 1;
}
//End Template Code: Functions
