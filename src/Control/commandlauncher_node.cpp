#include "commandlauncher_node.h"
//Start User Code: Firmware Definition
#define COMMANDLAUNCHERNODE_MAJOR_RELEASE 0
#define COMMANDLAUNCHERNODE_MINOR_RELEASE 0
#define COMMANDLAUNCHERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_loop1_code()
{
	logger->log_info("Running Loop1");
	std::vector<ProcessCommand> processlist = process->get_processlist();
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		bool check_pid = false;
		if(processlist.at(i).restart_counter >= MAX_PROCESS_RESTARTS)
		{
			char tempstr[512];
			sprintf(tempstr,"Process: %s restarted %d times.  Not restarting anymore.",processlist.at(i).name.c_str(),processlist.at(i).restart_counter);
			logger->log_warn(std::string(tempstr));
		}
		else if(processlist.at(i).running == false)
		{
			int ret = system (processlist.at(i).command_text.c_str());
			process->set_process_restarted(processlist.at(i).name);
			check_pid = true;
		}
		if((processlist.at(i).running == true) || (check_pid == true))
		{
			uint32_t pid = get_pid_byname(processlist.at(i).process_name);
			if(pid == 0) 
			{
				process->set_processrunning(processlist.at(i).name,false);
				char tempstr[512];
				sprintf(tempstr,"Process: %s is not running.  Restarting (%d) times so far.",processlist.at(i).name.c_str(),processlist.at(i).restart_counter);
				logger->log_warn(std::string(tempstr));
			}
			else
			{
				process->set_processpid(processlist.at(i).name,pid);
				process->set_processrunning(processlist.at(i).name,true);
			}
		}
	}
	return true;
}
bool run_loop2_code()
{
	icarus_rover_v2::diagnostic diag = process->update(measure_time_diff(ros::Time::now(),last_loop2_timer));
	if(diag.Level >= WARN)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
 	return true;
}
bool run_loop3_code()
{
 	return true;
}

void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_notice(process->get_processinfo());
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "commandlauncher_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 19-Sep-2017";
	fw.Major_Release = COMMANDLAUNCHERNODE_MAJOR_RELEASE;
	fw.Minor_Release = COMMANDLAUNCHERNODE_MINOR_RELEASE;
	fw.Build_Number = COMMANDLAUNCHERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
    if(device_initialized == true)
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
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
    /*
    {
    	icarus_rover_v2::diagnostic diag = process->send_querymessage(SPIMessageHandler::SPI_TestMessageCounter_ID);
    	if(diag.Level >= WARN)
    	{
    		diagnostic_pub.publish(diag);
    		logger->log_diagnostic(diag);
    	}
    }
	*/
}
std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");

	if(status == true)
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}

void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	//logger->log_info("Got command");
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
			std::vector<icarus_rover_v2::diagnostic> diaglist = check_program_variables();
			for(int i = 0; i < diaglist.size();i++) { diagnostic_pub.publish(diaglist.at(i)); }
		}
		else if(msg->Option1 == LEVEL3)
		{
		}
		else if(msg->Option1 == LEVEL4)
		{
		}
		else
		{
			logger->log_error("Shouldn't get here!!!");
		}
	}
}
uint32_t get_pid_byname(std::string name)
{
	char buf[512];
	char tempstr[256];
	sprintf(tempstr,"pidof -s %s",name.c_str());
	FILE *cmd_pipe = popen(tempstr, "r");

	fgets(buf, 512, cmd_pipe);
	pid_t pid = strtoul(buf, NULL, 10);
	pclose( cmd_pipe );
	return pid;
}
//End User Code: Functions
bool run_10Hz_code()
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

    if(diagnostic_status.Level > NOTICE)
    {
        diagnostic_pub.publish(diagnostic_status);
    }
    return true;
}
int main(int argc, char **argv)
{
	node_name = "commandlauncher_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    
    if(initializenode() == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
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
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = CONTROLLER_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,1000);

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
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = n->subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
    pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback);
    pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback);
    command_sub = n->subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    
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
        logger->log_warn("Missing parameter: loop1_rate.  Not running loop2 code.");
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
    process = new CommandLauncherNodeProcess;
	diagnostic_status = process->init(diagnostic_status,std::string(hostname));
	std::string param_camerastream = node_name + "/CameraStream";
	std::string camerastream;
	if(n->getParam(param_camerastream,camerastream) == false)
    {
        logger->log_error("Missing parameter: CameraStream.  Exiting.");
        return false;
    }
	if(process->set_camerastream(camerastream) == false)
	{
		logger->log_error("Couldn't find CameraStream in process list. Exiting.");
		return false;
	}
	
    //Finish User Code: Initialization and Parameters

    //Start Template Code: Final Initialization.
	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Initialized";
	diagnostic_pub.publish(diagnostic_status);
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
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	if(process->is_ready() == false)
	{
		logger->log_notice("Device not initialized yet.");
		icarus_rover_v2::device newdevice;
		newdevice.Architecture = msg->Architecture;
		newdevice.BoardCount = msg->BoardCount;
		newdevice.Capabilities = msg->Capabilities;
		newdevice.DeviceName = msg->DeviceName;
		newdevice.DeviceParent = msg->DeviceParent;
		newdevice.DeviceType = msg->DeviceType;
		newdevice.ID = msg->ID;
		newdevice.SensorCount = msg->SensorCount;
		newdevice.ShieldCount = msg->ShieldCount;
		newdevice.pins = msg->pins;
		diagnostic_status = process->new_devicemsg(newdevice);
		logger->log_diagnostic(diagnostic_status);
		if(diagnostic_status.Level > INFO) { diagnostic_pub.publish(diagnostic_status); }
		if(process->is_ready() == true)
		{
			myDevice = process->get_mydevice();
			resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
			logger->log_notice("Device finished initializing.");
		}
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
