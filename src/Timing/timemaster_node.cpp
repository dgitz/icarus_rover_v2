#include "timemaster_node.h"
//Start User Code: Firmware Definition
#define TIMEMASTERNODE_MAJOR_RELEASE 0
#define TIMEMASTERNODE_MINOR_RELEASE 0
#define TIMEMASTERNODE_BUILD_NUMBER 1
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_1000Hz_code()
{
    pps1000_delay = (pps1000_delay + measure_time_diff(ros::Time::now(),last_pps1000_timer))/2.0;
    last_pps1000_timer = ros::Time::now();
	std_msgs::Bool msg;
	msg.data = true;
	pps1000_pub.publish(msg);
	return true;
}
bool run_100Hz_code()
{
    pps100_delay = (pps100_delay + measure_time_diff(ros::Time::now(),last_pps100_timer))/2.0;
    last_pps100_timer = ros::Time::now();
	std_msgs::Bool msg;
	msg.data = true;
	pps100_pub.publish(msg);
	return true;
}
bool run_10Hz_code()
{
    pps10_delay = (pps10_delay + measure_time_diff(ros::Time::now(),last_pps10_timer))/2.0;
    last_pps10_timer = ros::Time::now();
    double delay_perc = 100.0*((pps10_delay - 0.1)/0.1);

    if(abs(delay_perc) > 130.0)
    {
        icarus_rover_v2::diagnostic diag = diagnostic_status;
        char tempstr[512];
        sprintf(tempstr,"Timing is off by: %f (sec)/%f%",pps10_delay-0.1,delay_perc);
        diag.Diagnostic_Type = TIMING;
		diag.Level = WARN;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = std::string(tempstr);
        logger->log_warn(std::string(tempstr));
    }
    else if(abs(delay_perc) > 300.0)
    {
        icarus_rover_v2::diagnostic diag = diagnostic_status;
        char tempstr[512];
        sprintf(tempstr,"Timing is off by: %f (sec)/%f%",pps10_delay-0.1,delay_perc);
        diag.Diagnostic_Type = TIMING;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		diag.Description = std::string(tempstr);
        logger->log_error(std::string(tempstr));
    }

	std_msgs::Bool msg;
	msg.data = true;
	pps10_pub.publish(msg);
	return true;
}
bool run_1Hz_code()
{
    pps1_delay = (pps1_delay + measure_time_diff(ros::Time::now(),last_pps1_timer))/2.0;
    last_pps1_timer = ros::Time::now();
	std_msgs::Bool msg;
	msg.data = true;
	pps1_pub.publish(msg);
	return true;
}
bool run_01Hz_code()
{
    pps01_delay = (pps01_delay + measure_time_diff(ros::Time::now(),last_pps01_timer))/2.0;
    last_pps01_timer = ros::Time::now();
	std_msgs::Bool msg;
	msg.data = true;
	pps01_pub.publish(msg);
	return true;
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "timemaster_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 7-April-2017";
	fw.Major_Release = TIMEMASTERNODE_MAJOR_RELEASE;
	fw.Minor_Release = TIMEMASTERNODE_MINOR_RELEASE;
	fw.Build_Number = TIMEMASTERNODE_BUILD_NUMBER;
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
}
void PPS10_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
}
void PPS100_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    //diagnostic_status = process->update(0.01);
    //diagnostic_pub.publish(diagnostic_status);
}
void PPS1000_Callback(const std_msgs::Bool::ConstPtr& msg)
{
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
//End User Code: Functions

int main(int argc, char **argv)
{
	node_name = "timemaster_node";
    ros::init(argc, argv, node_name);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    
    if(initialize(n) == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
		kill_node = 1;
    }
    ros::Rate loop_rate(10000);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    pps01_timer = pps1_timer = pps10_timer = pps100_timer = pps1000_timer = now;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,pps1000_timer);
    		if(mtime > .001)
    		{
    			run_1000Hz_code();
    			pps1000_timer = ros::Time::now();
    		}

    		mtime = measure_time_diff(now,pps100_timer);
    		if(mtime > .01)
    		{
    			run_100Hz_code();
    			pps100_timer = ros::Time::now();
    		}

    		mtime = measure_time_diff(now,pps10_timer);
    		if(mtime > .1)
    		{
                
    			run_10Hz_code();
    			pps10_timer = ros::Time::now();
    		}

    		mtime = measure_time_diff(now,pps1_timer);
    		if(mtime > 1.0)
    		{
    			if(publish_1pps == true)
    			{
    				run_1Hz_code();
    			}
    			pps1_timer = ros::Time::now();
    		}

    		mtime = measure_time_diff(now,pps01_timer);
    		if(mtime > 10.0)
    		{
    			run_01Hz_code();
    			pps01_timer = ros::Time::now();
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
bool initialize(ros::NodeHandle nh)
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
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = TIMING_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

    std::string param_verbosity_level = node_name +"/verbosity_level";
    if(nh.getParam(param_verbosity_level,verbosity_level) == false)
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
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
    pps01_sub = nh.subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback); 
    pps1_sub = nh.subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback); 
    pps10_sub = nh.subscribe<std_msgs::Bool>("/10PPS",1000,PPS10_Callback); 
    pps100_sub = nh.subscribe<std_msgs::Bool>("/100PPS",1000,PPS100_Callback); 
    pps1000_sub = nh.subscribe<std_msgs::Bool>("/1000PPS",1000,PPS1000_Callback); 
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    //End Template Code: Initialization and Parameters
    //Start User Code: Initialization and Parameters
    pps01_delay = 0.0;
    pps1_delay = 0.0;
    pps10_delay = 0.0;
    pps100_delay = 0.0;
    pps1000_delay = 0.0;
    std::string pps_source;
    std::string param_pps_source = node_name +"/pps_source";
    if(nh.getParam(param_pps_source,pps_source) == false)
    {
    	logger->log_warn("Missing Parameter: pps_source.  Using default: self");
    	pps_source = "self";
    }
    if(pps_source == "self")
    {
    	publish_1pps = true;
		std::string pps1_topic = "/1PPS";
		pps1_pub =  nh.advertise<std_msgs::Bool>(pps1_topic,10);
    }
    else
    {
    	publish_1pps = false;
    	char tempstr[512];
    	sprintf(tempstr,"PPS Source: %s Not Supported",pps_source.c_str());
    	logger->log_error(std::string(tempstr));
    }
    last_pps01_timer = ros::Time::now();
    last_pps1_timer = ros::Time::now();
    last_pps10_timer = ros::Time::now();
    last_pps100_timer = ros::Time::now();
    last_pps1000_timer = ros::Time::now();
    pps01_pub =  nh.advertise<std_msgs::Bool>("/01PPS",10);
    pps10_pub =  nh.advertise<std_msgs::Bool>("/10PPS",10);
    pps100_pub =  nh.advertise<std_msgs::Bool>("/100PPS",10);
    pps1000_pub =  nh.advertise<std_msgs::Bool>("/1000PPS",10);
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
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;

	if((newdevice.DeviceName == hostname) && (device_initialized == false))
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
