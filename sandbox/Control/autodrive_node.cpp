#include "autodrive_node.h"
//Start User Code: Firmware Definition
#define AUTODRIVENODE_MAJOR_RELEASE 0
#define AUTODRIVENODE_MINOR_RELEASE 0
#define AUTODRIVENODE_BUILD_NUMBER 2
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	icarus_rover_v2::diagnostic diagnostic = process->update(measure_time_diff(ros::Time::now(),last_loop1_timer));
	if(diagnostic.Level > INFO)
	{
		logger->log_diagnostic(diagnostic);
	}

	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
	for(std::size_t i = 0; i < pin_topics.size(); i++)
	{
		std::vector<icarus_rover_v2::pin> pins = process->get_controlgroup_pins(pin_topics.at(i));
		for(std::size_t j = 0; j < pins.size(); j++)
		{
			pin_pubs.at(i).publish(pins.at(j));
		}
	}

 	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{
 	return true;
}

void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "autodrive_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 28-December-2017";
	fw.Major_Release = AUTODRIVENODE_MAJOR_RELEASE;
	fw.Minor_Release = AUTODRIVENODE_MINOR_RELEASE;
	fw.Build_Number = AUTODRIVENODE_BUILD_NUMBER;
	firmware_pub.publish(fw);

	if((process->is_initialized() == true) and (node_initialized == false))
	{
		std::vector<ControlGroup> controlgroups = process->get_controlgroups();
		for(std::size_t i = 0; i < controlgroups.size(); i++)
		{
			printf("created publisher: %s\n",controlgroups.at(i).output.topic.c_str());
			ros::Publisher pin_pub = n->advertise<icarus_rover_v2::pin>(controlgroups.at(i).output.topic,5);
			pin_topics.push_back(controlgroups.at(i).output.topic);
			pin_pubs.push_back(pin_pub);
		}
		node_initialized = true;
		logger->log_notice("Node Initialization and Process Initialization complete.");
	}
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
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
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
	else
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
/*! \brief Self-Diagnostic-Check Program Variables
 */
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
void ControlGroup_Callback(const icarus_rover_v2::controlgroup::ConstPtr& msg)
{
	icarus_rover_v2::controlgroup data;
	data.name = msg->name;
	data.type = msg->type;
	data.value1 = msg->value1;
	data.value2 = msg->value2;
	data.value3 = msg->value3;
	data.maxvalue = msg->maxvalue;
	data.minvalue = msg->minvalue;
	data.defaultvalue = msg->defaultvalue;
	icarus_rover_v2::diagnostic diagnostic = process->new_tunecontrolgroupmsg(data);
	if(diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
	}

}
void Pose_Callback(const icarus_rover_v2::pose::ConstPtr& msg)
{
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(controlgroups.at(i).sensor.name == SENSOR_POSEYAWRATE)
		{
			icarus_rover_v2::diagnostic diagnostic = process->new_sensormsg(controlgroups.at(i).name, msg->yawrate.value);
			if(diagnostic.Level > NOTICE)
			{
				logger->log_diagnostic(diagnostic);
			}
		}
	}
}
void Joy_Callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(controlgroups.at(i).command.name == "axis")
		{
			icarus_rover_v2::diagnostic diagnostic = process->new_commandmsg(controlgroups.at(i).name, msg->axes[controlgroups.at(i).command.index]);
			if(diagnostic.Level > NOTICE)
			{
				logger->log_diagnostic(diagnostic);
			}
		}
	}
}
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	diagnostic_status = process->new_armedstatemsg((uint8_t)msg->data);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
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
	node_name = "autodrive_node";
    ros::init(argc, argv, node_name);
    node_name = ros::this_node::getName();
    n.reset(new ros::NodeHandle);
    
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
	diagnostic_status.Component = TIMING_NODE;

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
	node_initialized = false;
	pin_pubs.clear();
	pin_topics.clear();
	CGSensorSubs.clear();
	CGCommandSubs.clear();
    process = new AutoDriveNodeProcess;
	diagnostic_status = process->init(diagnostic_status,std::string(hostname));

	std::string param_Mode = node_name +"/Mode";
	std::string Mode;
	if(n->getParam(param_Mode,Mode) == false)
	{
		logger->log_error("Missing Parameter: Mode.");
		return false;
	}
	if(Mode == "DriverStation")
	{
		std::string controlgroup_topic = "/" + Mode + "/controlgroup";
		controlgroup_sub = n->subscribe<icarus_rover_v2::controlgroup>(controlgroup_topic,5,ControlGroup_Callback);
	}
	else if(Mode == "Disabled")
	{
		printf("[autodrive node]Mode: Disabled.  Exiting Cleanly.");
		logger->log_notice("[autodrive node]Mode: Disabled.  Exiting Cleanly.");
		return false;
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"Mode: %s Not Supported.",Mode.c_str());
		logger->log_error(tempstr);
		return false;
	}
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	for(std::size_t i = 0; i < controlgroups.size(); i++)
	{
		if(controlgroups.at(i).sensor.type == "icarus_rover_v2/pose")
		{
			CGSensorSub sensor;
			sensor.type = "icarus_rover_v2/pose";
			sensor.topic = "/pose";
			sensor.sub = n->subscribe<icarus_rover_v2::pose>(sensor.topic,5,Pose_Callback); 
			CGSensorSubs.push_back(sensor);
		}
		else
		{
			char tempstr[256];
			sprintf(tempstr,"Sensor Type: %s not supported",controlgroups.at(i).sensor.type.c_str());
			logger->log_error(std::string(tempstr));
			return false;
		}
		
		if(controlgroups.at(i).command.type == "sensor_msgs/Joy")
		{
			CGCommandSub command;
			command.type = "sensor_msgs/Joy";
			command.topic = "/DriverStation/joystick";
			command.sub = n->subscribe<sensor_msgs::Joy>(command.topic,5,Joy_Callback); 
			CGCommandSubs.push_back(command);
		}
		else
		{
			char tempstr[256];
			sprintf(tempstr,"Command Type: %s not supported",controlgroups.at(i).command.type.c_str());
			logger->log_error(std::string(tempstr));
			return false;
		}
		
	}
	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,ArmedState_Callback);
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
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{

	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{
			myDevice = device;
			resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
			process->set_mydevice(device);
			device_initialized = true;
		}
	}

	if((device_initialized == true))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
		if(process->get_initialized() == true)
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
