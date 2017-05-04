#include "powermonitor_node.h"
//Start User Code: Firmware Definition
#define POWERMONITORNODE_MAJOR_RELEASE 3
#define POWERMONITORNODE_MINOR_RELEASE 1
#define POWERMONITORNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
icarus_rover_v2::diagnostic rescan_topics()
{
    icarus_rover_v2::diagnostic diag = diagnostic_status;
	int found_new_topics = 0;
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	std::vector<std::string> topics_to_add;
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "icarus_rover_v2/pin")
		{
			bool add_me = true;
			for(int i = 0; i < analogpin_topics.size();i++)
			{
				if(analogpin_topics.at(i) == info.name)
				{
					add_me = false;
					break;
				}
			}
			if(add_me == true)
			{
				analogpin_topics.push_back(info.name);
			}
		}
	}
	for(int i = 0; i < topics_to_add.size(); i++)
	{
        //ros::Subscriber analogpin_sub = n->subscribe<icarus_rover_v2::pin>(topics_to_add.at(i),1000,Pin_Callback);
        ros::Subscriber analogpin_sub = n->subscribe<icarus_rover_v2::pin>(topics_to_add.at(i),5,Pin_Callback);
        analogpin_subs.push_back(analogpin_sub);
	}

	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	if(topics_to_add.size() > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",(int)topics_to_add.size());
	}
	else
	{
		sprintf(tempstr,"Rescanned and found no new topics.");
	}
	logger->log_info(tempstr);
	diag.Description = tempstr;
	return diag;
}

void Pin_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
    icarus_rover_v2::diagnostic diag;
    icarus_rover_v2::pin newpin;
    newpin.BoardID = msg->BoardID;
    newpin.ShieldID = msg->ShieldID;
    newpin.PortID = msg->PortID;
    newpin.Number = msg->Number;
    newpin.Function = msg->Function;
    newpin.Value = msg->Value;
    newpin.DefaultValue = msg->DefaultValue;
    newpin.ConnectedDevice = msg->ConnectedDevice;
    newpin.ADCResolution = msg->ADCResolution;
    newpin.VoltageReference = msg->VoltageReference;
    diag = process->new_pinmsg(newpin);
    if(diag.Level > NOTICE)
    {
        diagnostic_pub.publish(diag);
    }
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    diagnostic_status = rescan_topics();
    if(diagnostic_status.Level > INFO)
    {
        diagnostic_pub.publish(diagnostic_status);
    }
    logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "powermonitor_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 3-May-2017";
	fw.Major_Release = POWERMONITORNODE_MAJOR_RELEASE;
	fw.Minor_Release = POWERMONITORNODE_MINOR_RELEASE;
	fw.Build_Number = POWERMONITORNODE_BUILD_NUMBER;
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
    logger->log_info(process->print_batteryinfo());
}
void PPS10_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	std::vector<icarus_rover_v2::battery> batteries = process->get_batteries();
    for(int i = 0; i < batteries.size(); i++)
    {
        battery_pub.publish(batteries.at(i));
    }
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
    diagnostic_status = process->update(0.01);
    if(diagnostic_status.Level > INFO) { diagnostic_pub.publish(diagnostic_status); }
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
int main(int argc, char **argv)
{
	node_name = "io_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();
    
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
    ros::Rate loop_rate(1);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
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
	diagnostic_status.Component = GPIO_NODE;

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
    pps10_sub = n->subscribe<std_msgs::Bool>("/10PPS",1000,PPS10_Callback); 
    pps100_sub = n->subscribe<std_msgs::Bool>("/100PPS",1000,PPS100_Callback); 
    pps1000_sub = n->subscribe<std_msgs::Bool>("/1000PPS",1000,PPS1000_Callback); 
    command_sub = n->subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
	process = new PowerMonitorNodeProcess;
	diagnostic_status = process->init(diagnostic_status,std::string(hostname));
    
    std::string battery_topic = "/" + node_name + "/battery";
    battery_pub = n->advertise<icarus_rover_v2::battery>(battery_topic,1000);
    
    
    analogpin_topics.clear();
    analogpin_subs.clear();
   
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
	newdevice.DeviceType = msg->DeviceType;
	newdevice.DeviceParent = msg->DeviceParent;
	newdevice.BoardCount = msg->BoardCount;
	newdevice.SensorCount = msg->SensorCount;
	newdevice.pins = msg->pins;
	diagnostic_status = process->new_devicemsg(newdevice);
	if(diagnostic_status.Level == FATAL)
	{
		logger->log_fatal(diagnostic_status.Description);
		logger->log_fatal("This is a Safety Issue!  Killing Node.");
		kill_node = true;
	}
	if((device_initialized == false) && (process->is_finished_initializing()))
	{
		myDevice = process->get_mydevice();
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
