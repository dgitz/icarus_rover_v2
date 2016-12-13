#include "teleop_node.h"
//Start User Code: Firmware Definition
#define TELEOPNODE_MAJOR_RELEASE 1
#define TELEOPNODE_MINOR_RELEASE 1
#define TELEOPNODE_BUILD_NUMBER 1
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	if((armed_state == ARMEDSTATUS_ARMED) && (arm_command == ARMEDCOMMAND_ARM))
	{
		rc_pub.publish(rc_command);
	}
	else
	{
		rc_pub.publish(defaultrc_command);
	}
	return true;
}
bool run_mediumrate_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
int map_input_to_output(double x, double deadband,int min, int neutral, int max)
{
	if(fabs(x) <= deadband)
	{
		return neutral;
	}
	else if(x < 0)
	{
		double m = ((double)(neutral-min))/(-deadband-(-1.0));
		return (int)(m*(x-(-1.0))+min);
	}
	else if(x > 0)
	{
		double m = ((double)(max-neutral))/(1.0-deadband);
		return (int)(m*(x-1.0)+max);
	}
	else
	{
		return neutral;
	}
}
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	armed_state = msg->data;
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
	//logger->log_debug("Running slow rate code.");

	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "teleop_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 11-Dec-2016";
	fw.Major_Release = TELEOPNODE_MAJOR_RELEASE;
	fw.Minor_Release = TELEOPNODE_MINOR_RELEASE;
	fw.Build_Number = TELEOPNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
void Joystick_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	for(int j = 0; j < joy->axes.size();j++)
	{
		for(int c = 0; c < Channels.size();c++)
		{
			if(Channels.at(c).input_channel == j)
			{
				double v_in = joy->axes[j];
				if(Channels.at(c).inverted == 1){ v_in = -1.0*v_in; }

				int v = map_input_to_output(v_in,Channels.at(c).deadband,Channels.at(c).min_value,Channels.at(c).neutral_value,Channels.at(c).max_value);
				rc_command.channel[Channels.at(c).output_channel] = v;
				//printf("j: %d v: %f output: %d %d\n",j,v_in,Channels.at(c).output_channel,v);
			}
		}
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
//End User Code: Functions

int main(int argc, char **argv)
{
	node_name = "sample_node";
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
    ros::Rate loop_rate(rate);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .02)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
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
    for(int i = 0; i < 8; i++)
    {
    	rc_command.channel[i] = 1500;
    	defaultrc_command.channel[i] = 1500;
    }
    std::string armed_state_topic = "/armed_state";
    armed_state_sub = nh.subscribe<std_msgs::UInt8>(armed_state_topic,1000,ArmedState_Callback);
    std::string joy_topic;
    std::string param_joy_topic = node_name +"/joystick_topic";
    if(nh.getParam(param_joy_topic,joy_topic) == false)
    {
    	logger->log_warn("Missing Parameter: joystick_topic.");
    	return false;
    }
    joy_sub = nh.subscribe<sensor_msgs::Joy>(joy_topic,1000,Joystick_Callback);
    rc_pub = nh.advertise<icarus_rover_v2::remotecontrol>("/remote_control",1000);
    for(int i = 0; i < 8; i++)
    {
    	std::string param_channelid = node_name +"/Channel" + boost::lexical_cast<std::string>(i) + "_Name";
    	std::string channel_name;
    	if(nh.getParam(param_channelid,channel_name) == true)
    	{
    		Channel newch;
    		newch.Name = channel_name;
    		newch.input_channel = i;
    		int inverted,minv,maxv,neutralv,outputchannel;
    		double deadband;
    		std::string param_channel_inverted = node_name +"/" + newch.Name + "_Inverted";
    		std::string param_channel_minvalue = node_name +"/" + newch.Name + "_MinValue";
    		std::string param_channel_neutralvalue = node_name +"/" + newch.Name + "_NeutralValue";
    		std::string param_channel_manvalue = node_name +"/" + newch.Name + "_MaxValue";
    		std::string param_channel_output = node_name + "/" + newch.Name + "_OutputChannel";
    		std::string param_channel_deadband = node_name + "/" + newch.Name + "_Deadband";
    		if(nh.getParam(param_channel_inverted,inverted) == false)
			{
    			char tempstr[1024];
    			sprintf(tempstr,"Missing Parameter: %s.  Exiting.",param_channel_inverted.c_str());
    			logger->log_error(tempstr);
    			return false;
			}
    		if(nh.getParam(param_channel_minvalue,minv) == false)
    		{
    			char tempstr[1024];
    			sprintf(tempstr,"Missing Parameter: %s.  Exiting.",param_channel_minvalue.c_str());
    			logger->log_error(tempstr);
    			return false;
    		}
    		if(nh.getParam(param_channel_neutralvalue,neutralv) == false)
    		{
    			char tempstr[1024];
    			sprintf(tempstr,"Missing Parameter: %s.  Exiting.",param_channel_neutralvalue.c_str());
    			logger->log_error(tempstr);
    			return false;
    		}
    		if(nh.getParam(param_channel_manvalue,maxv) == false)
    		{
    			char tempstr[1024];
    			sprintf(tempstr,"Missing Parameter: %s.  Exiting.",param_channel_manvalue.c_str());
    			logger->log_error(tempstr);
    			return false;
    		}
    		if(nh.getParam(param_channel_output,outputchannel) == false)
    		{
    			char tempstr[1024];
    			sprintf(tempstr,"Missing Parameter: %s.  Exiting.",param_channel_output.c_str());
    			logger->log_error(tempstr);
    			return false;
    		}
    		if(nh.getParam(param_channel_deadband,deadband) == false)
    		{
    			char tempstr[1024];
    			sprintf(tempstr,"Missing Parameter: %s.  Exiting.",param_channel_deadband.c_str());
    			logger->log_error(tempstr);
    			return false;
    		}
    		newch.inverted = inverted;
    		newch.min_value = minv;
    		newch.neutral_value = neutralv;
    		newch.max_value = maxv;
    		newch.output_channel = outputchannel;
    		newch.deadband = deadband;
    		Channels.push_back(newch);
    	}
    }
    for(int c = 0; c < Channels.size(); c++)
    {
    	defaultrc_command.channel[Channels.at(c).input_channel] = Channels.at(c).neutral_value;

    	printf("Input Channel: %d Name: %s Deadband: %f Inverted: %d Min: %d Neutral: %d Max: %d Output: %d\n",
    			Channels.at(c).input_channel,
				Channels.at(c).Name.c_str(),
				Channels.at(c).deadband,
				Channels.at(c).inverted,
				Channels.at(c).min_value,
				Channels.at(c).neutral_value,
				Channels.at(c).max_value,
				Channels.at(c).output_channel);
    }
    for(int j = 0; j < 8;j++)
    {
    	for(int c = 0; c < Channels.size();c++)
    	{
    		if(Channels.at(c).input_channel == j)
    		{
    			rc_command.channel[Channels.at(c).output_channel] = Channels.at(c).neutral_value;
    		}
    	}
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
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
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
