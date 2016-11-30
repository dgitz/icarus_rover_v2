#include "io_node.h"
//Start Template Code: Firmware Definition
#define IONODE_MAJOR_RELEASE 2
#define IONODE_MINOR_RELEASE 3
#define IONODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
//Start User Code: Functions

bool run_fastrate_code()
{
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

	return true;
}
bool run_mediumrate_code()
{

	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);

	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_slowrate_code()
{


	if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic;
		resource_diagnostic = resourcemonitor->update();
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


	return true;
}

bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "io_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 24-Nov-2016";
	fw.Major_Release = IONODE_MAJOR_RELEASE;
	fw.Minor_Release = IONODE_MINOR_RELEASE;
	fw.Build_Number = IONODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
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

//End User Code: Functions

//Start Template Code: Functions
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	char tempstr[128];
	sprintf(tempstr,"Got Command: %0X Level: %d",msg->Command,msg->Option1);
	logger->log_info(tempstr);
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
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
        return 0; 
    }
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
	boot_time = now;
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok())
    {
		if(kill_node == true)
		{
			return 0;
		}
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .001)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
				//printf("Executing\n");
				run_mediumrate_code();
				medium_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,slow_timer);
			if(mtime > 1.0)
			{
				run_slowrate_code();
				slow_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,veryslow_timer);
			if(mtime > 10.0)
			{
				run_veryslowrate_code();
				veryslow_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
    logger->log_notice("Node Finished cleanly.  Disabling all Actuators.");
    diagnostic_status = process->enable_actuators(false);
    diagnostic_pub.publish(diagnostic_status);
    logger->log_notice("Node Finished Safely.");
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization and Parameters
	kill_node = false;
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    myDevice.DeviceParent = "";
    myDevice.DeviceType = "";
    device_initialized = false;
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    process = new IONodeProcess;

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
    diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname));
    std::string param_loop_rate = node_name +"/loop_rate";
    if(nh.getParam(param_loop_rate,rate) == false)
    {
        logger->log_warn("Missing Parameter: loop_rate.");
        return false;
    }

	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;

    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	
    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
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
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
	received_pps = true;
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
//End Template Code: Functions
