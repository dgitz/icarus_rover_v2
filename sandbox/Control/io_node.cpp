OBSOLETE!!!
#include "io_node.h"
//Start User Code: Firmware Definition
#define IONODE_MAJOR_RELEASE 2
#define IONODE_MINOR_RELEASE 3
#define IONODE_BUILD_NUMBER 2
//End User Code: Firmware Definition
//Start User Code: Functions
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "io_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 30-Nov-2016";
	fw.Major_Release = IONODE_MAJOR_RELEASE;
	fw.Minor_Release = IONODE_MINOR_RELEASE;
	fw.Build_Number = IONODE_BUILD_NUMBER;
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
	diagnostic_status = process->update(20,remote_armed_state,arm_command);  //Need to change 20 to the actual dt!!!
	local_armed_state = process->get_armedstate();
	if(diagnostic_status.Level > NOTICE)
	{
		logger->log_warn(diagnostic_status.Description);
		diagnostic_pub.publish(diagnostic_status);
	}
	Port_Info Port;
	Port = process->get_PortInfo("GPIO_Port");
	for(int i = 0; i < 32; i++)
	{
		if(Port.Available[i] == true)
		{
			if(Port.Mode[i] == PINMODE_DIGITAL_INPUT)
			{
				icarus_rover_v2::pin newpin;
				newpin.ConnectedDevice = Port.ConnectingDevice.at(i);
				newpin.Function = process->map_PinFunction_ToString(Port.Mode[i]);
				newpin.Port = "GPIO_Port;";
				newpin.Number = i+1;
				newpin.Value = Port.Value[i];
				digitalinput_pub.publish(newpin);
			}
		}
	}
	if((arm_command == ARMEDCOMMAND_ARM) and
	   (local_armed_state == ARMEDSTATUS_ARMED))
	{
		enable_actuators = true;

	}
	else
	{
		enable_actuators = false;
		diagnostic_status = process->enable_actuators(enable_actuators);
		if(diagnostic_status.Level > NOTICE)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
	}
	if(last_enable_actuators != enable_actuators)
	{
		diagnostic_status = process->enable_actuators(enable_actuators);
		diagnostic_pub.publish(diagnostic_status);
		last_enable_actuators =  enable_actuators;
	}

}
void PPS1000_Callback(const std_msgs::Bool::ConstPtr& msg)
{
}

void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	remote_armed_state = msg->data;
}

void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	icarus_rover_v2::pin pinmsg;
	pinmsg.Port = msg->Port;
	pinmsg.Function = msg->Function;
	pinmsg.Number = msg->Number;
	pinmsg.Value = msg->Value;
	pinmsg.ConnectedDevice = msg->ConnectedDevice;
	if(pinmsg.Function == "DigitalOutput")
	{
		//char tempstr[128];
		//sprintf(tempstr,"Pin: %d Mode: %s Value: %d",pinmsg.Number,pinmsg.Function.c_str(),pinmsg.Value);
		//logger->log_debug(tempstr);
		process->new_pinmsg(pinmsg);
	}
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
	else if(msg->Command == ARM_COMMAND_ID)
	{
		arm_command = msg->Option1;
	}
}
int main(int argc, char **argv)
{
	node_name = "io_node";
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
    		mtime = measure_time_diff(now,fast_timer);
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
	logger->log_notice("Node Exiting. Disabling Actuators");
    diagnostic_status = process->enable_actuators(false);
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
	diagnostic_status.Component = GPIO_NODE;

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
	process = new IONodeProcess;
	diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname));
    remote_armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    local_armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    arm_command = ARMEDCOMMAND_DISARM;
    enable_actuators = false;
    last_enable_actuators = enable_actuators;

    std::string digitalinput_topic = "/" + node_name + "/DigitalInput";
    digitalinput_pub = nh.advertise<icarus_rover_v2::pin>(digitalinput_topic,1000);

    std::string digitaloutput_topic = "/" + node_name + "/DigitalOutput";
    digitaloutput_sub = nh.subscribe<icarus_rover_v2::pin>(digitaloutput_topic,1000,DigitalOutput_Callback);


    std::string armed_state_topic = "/armed_state";
    armed_state_sub = nh.subscribe<std_msgs::UInt8>(armed_state_topic,1000,ArmedState_Callback);
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
