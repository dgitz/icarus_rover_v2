#include "command_node.h"
//Start User Code: Firmware Definition
#define COMMANDNODE_MAJOR_RELEASE 3
#define COMMANDNODE_MINOR_RELEASE 0
#define COMMANDNODE_BUILD_NUMBER 1
//End User Code: Firmware Definition
//Start User Code: Functions
void ReadyToArm_Callback(const std_msgs::Bool::ConstPtr& msg,const std::string &topic)
{
    diagnostic_status = process->new_readytoarmmsg(topic,msg->data);
    if(diagnostic_status.Level > INFO)
    {
    	diagnostic_pub.publish(diagnostic_status);
    }
}
void User_ArmCommand_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
    diagnostic_status = process->new_user_armcommandmsg(msg->data);
    diagnostic_pub.publish(diagnostic_status);
}
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	diagnostic_status = process->update(.02);
	if(process->get_currentcommand().Command != ROVERCOMMAND_NONE)
	{
		command_pub.publish(process->get_currentcommand());
	}
	std_msgs::UInt8 state;
	state.data = process->get_armeddisarmed_state();
	armeddisarmed_state_pub.publish(state);
	return true;
}
bool run_mediumrate_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	//logger->log_debug("Running medium rate code.");
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
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
	//logger->log_debug("Running slow rate code.");

	return true;
}
bool run_veryslowrate_code()
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "command_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 3-March-2017";
	fw.Major_Release = COMMANDNODE_MAJOR_RELEASE;
	fw.Minor_Release = COMMANDNODE_MINOR_RELEASE;
	fw.Build_Number = COMMANDNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
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
//End User Code: Functions

int main(int argc, char **argv)
{
	node_name = "command_node";
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
	diagnostic_status.Component = CONTROLLER_NODE;

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
    searchmode = 0;
	process = new CommandNodeProcess;
    robot_armdisarmed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
    diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname));
    std::string command_topic = "/command";
    command_pub =  nh.advertise<icarus_rover_v2::command>(command_topic,1000);

    std::vector<std::string> ready_to_arm_topics;

    bool search_for_topics = true;
    int topicindex = 1;
    while(search_for_topics == true)
    {
    	std::string topic;
    	bool add_new_topic = false;
    	std::string param_topic = node_name +"/ready_to_arm_topic" + boost::lexical_cast<std::string>(topicindex);
    	if(nh.getParam(param_topic,topic) == false)
    	{
    		char tempstr[255];
    		sprintf(tempstr,"Didn't find %s Not adding anymore.",param_topic.c_str());
    		logger->log_info(tempstr);
    		add_new_topic = false;
    		search_for_topics = false;
    	}
    	else
    	{
    		char tempstr[255];
    		sprintf(tempstr,"Adding Ready to Arm Topic: %s",topic.c_str());
    		logger->log_info(tempstr);
    		ready_to_arm_topics.push_back(topic);
    		search_for_topics = true;
    		topicindex++;
    	}
    }
    for(int i = 0; i < ready_to_arm_topics.size();i++)
    {
        ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(ready_to_arm_topics.at(i),1000,boost::bind(ReadyToArm_Callback,_1,ready_to_arm_topics.at(i)));
        ready_to_arm_subs.push_back(sub);
    } 
    diagnostic_status = process->init_readytoarm_list(ready_to_arm_topics);
    if(diagnostic_status.Level >= WARN)
    {
        logger->log_warn("Unable to initialize Ready To Arm List.  Exiting.");
        return false;
    }

    std::string param_user_armcommand_topic = node_name +"/user_armcommand_topic";
    std::string armcommand_topic;
    if(nh.getParam(param_user_armcommand_topic,armcommand_topic) == false)
    {
        logger->log_error("Missing parameter: user_armcommand_topic. Exiting.");
        return false;
    }
    armcommand_sub = nh.subscribe<std_msgs::UInt8>(armcommand_topic,1000,User_ArmCommand_Callback);
    
    std::string armeddisarmed_state_topic = "/armed_state";
    armeddisarmed_state_pub = nh.advertise<std_msgs::UInt8>(armeddisarmed_state_topic,1000);

    //Initialize Periodic Commands
    std::vector<PeriodicCommand> pcommands;
    icarus_rover_v2::command command1;
    command1.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
    command1.Option1 = LEVEL1;
    command1.Description = "Low-Level Diagnostics";
    PeriodicCommand pcommand1;
    pcommand1.command = command1;
    pcommand1.rate_hz = 10.0;
    pcommands.push_back(pcommand1);

    icarus_rover_v2::command command2;
    command2.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
    command2.Option1 = LEVEL2;
    command2.Description = "Mid-Level Diagnostics";
    PeriodicCommand pcommand2;
    pcommand2.command = command2;
    pcommand2.rate_hz = 1.0;
    pcommands.push_back(pcommand2);

    icarus_rover_v2::command command3;
    command3.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
    command3.Option1 = LEVEL3;
    command3.Description = "High-Level Diagnostics";
    PeriodicCommand pcommand3;
    pcommand3.command = command3;
    pcommand3.rate_hz = 0.1;
    pcommands.push_back(pcommand3);
    diagnostic_status = process->init_PeriodicCommands(pcommands);
    if(diagnostic_status.Level > NOTICE)
    {
    	logger->log_error(diagnostic_status.Description);
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
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.DeviceParent = msg->DeviceParent;
	newdevice.DeviceType = msg->DeviceType;
	newdevice.Architecture = msg->Architecture;
	process->new_devicemsg(newdevice);
	if(diagnostic_status.Level == FATAL)
	{
		logger->log_fatal(diagnostic_status.Description);
		logger->log_fatal("This is a Safety Issue!  Killing Node.");
		kill_node = 1;
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
