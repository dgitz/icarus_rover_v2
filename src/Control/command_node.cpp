#include "command_node.h"
//Start User Code: Firmware Definition
#define COMMANDNODE_MAJOR_RELEASE 3
#define COMMANDNODE_MINOR_RELEASE 1
#define COMMANDNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_loop1_code()
{
	std_msgs::UInt8 state;
	state.data = process->get_armeddisarmed_state();
	armeddisarmed_state_pub.publish(state);
	return true;
}
bool run_loop2_code()
{
	diagnostic_status = process->update(0.05);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
 	return true;
}
bool run_loop3_code()
{
	if((process->get_currentcommand().Command == ROVERCOMMAND_ARM) ||
			(process->get_currentcommand().Command == ROVERCOMMAND_DISARM) ||
			(process->get_currentcommand().Command == ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY))
	{

		command_pub.publish(process->get_currentcommand());
	}
 	return true;
}


void ReadyToArm_Callback(const std_msgs::Bool::ConstPtr& msg,const std::string &topic)
{
    diagnostic_status = process->new_readytoarmmsg(topic,msg->data);
    if(diagnostic_status.Level > INFO)
    {
    	diagnostic_pub.publish(diagnostic_status);
    }
}
void User_Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::command command;
	command.Command = msg->Command;
	command.Option1 = msg->Option1;
	command.Option2 = msg->Option2;
	command.Option3 = msg->Option3;
	command.CommandText = msg->CommandText;
	command.Description = msg->Description;
    diagnostic_status = process->new_user_commandmsg(command);
    logger->log_diagnostic(diagnostic_status);
    diagnostic_pub.publish(diagnostic_status);
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "command_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 7-Aril-2017";
	fw.Major_Release = COMMANDNODE_MAJOR_RELEASE;
	fw.Minor_Release = COMMANDNODE_MINOR_RELEASE;
	fw.Build_Number = COMMANDNODE_BUILD_NUMBER;
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
			logger->log_diagnostic(resource_diagnostic);
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
    ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    last_10Hz_timer = ros::Time::now();
    double mtime;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
            if(run_loop1 == true)
            {
                mtime = measure_time_diff(now,last_loop1_timer);
                if(mtime >= (1.0/loop1_rate))
                {
                    run_loop1_code();
                    last_loop1_timer = ros::Time::now();
                }
            }
            if(run_loop2 == true)
            {
                mtime = measure_time_diff(now,last_loop2_timer);
                if(mtime >= (1.0/loop2_rate))
                {
                    run_loop2_code();
                    last_loop2_timer = ros::Time::now();
                }
            }
            if(run_loop3 == true)
            {
                mtime = measure_time_diff(now,last_loop3_timer);
                if(mtime >= (1.0/loop3_rate))
                {
                    run_loop3_code();
                    last_loop3_timer = ros::Time::now();
                }
            }
            
            mtime = measure_time_diff(now,last_10Hz_timer);
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
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
    pps01_sub = nh.subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback); 
    pps1_sub = nh.subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback); 
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
	
	double max_rate = 0.0;
    std::string param_loop1_rate = node_name + "/loop1_rate";
    if(nh.getParam(param_loop1_rate,loop1_rate) == false)
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
    if(nh.getParam(param_loop2_rate,loop2_rate) == false)
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
    if(nh.getParam(param_loop3_rate,loop3_rate) == false)
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

    std::string param_user_command_topic = node_name +"/user_command_topic";
    std::string user_command_topic;
    if(nh.getParam(param_user_command_topic,user_command_topic) == false)
    {
        logger->log_error("Missing parameter: user_command_topic. Exiting.");
        return false;
    }
    user_command_sub = nh.subscribe<icarus_rover_v2::command>(user_command_topic,1000,User_Command_Callback);
    
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
