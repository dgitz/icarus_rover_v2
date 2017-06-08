#include "speaker_node.h"
//Start User Code: Firmware Definition
#define SPEAKERNODE_MAJOR_RELEASE 2
#define SPEAKERNODE_MINOR_RELEASE 1
#define SPEAKERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_loop1_code() //!!!RUN AT 50 HZ
{
	diagnostic_status = speakernodeprocess->update(.02);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
	if(speakernodeprocess->readytospeak() == true)
	{
		std::string speechout = speakernodeprocess->get_speechoutput();
		sc->say(speechout);
		char tempstr[512];
		sprintf(tempstr,"Saying: %s",speechout.c_str());
		logger->log_debug(tempstr);
	}
	return true;
}
bool run_loop2_code()
{
 	return true;
}
bool run_loop3_code()
{
 	return true;
}
/*
bool speak(std::string s,bool mode)
{
	double runtime = measure_time_diff(ros::Time::now(),boot_time);
	if(runtime > initial_speech_wait)
	{
		if(mode == false) //periodic update mode
		{
			if(speechbuffer.size() > 0)
			{
				s = speechbuffer.at(0);
				double how_long_to_say_sec = (double)(s.size()/(double)SPEECH_RATE);
				double etime = measure_time_diff(ros::Time::now(),last_time_speech_ended);
				if(etime > MIN_TIME_BEFORE_SPEAK_AGAIN)
				{
					sc->say(s);
					last_time_speech_ended  = ros::Time::now() + ros::Duration(how_long_to_say_sec);
					speechbuffer.erase(speechbuffer.begin());
					if(speechbuffer.size() > MAX_SPEECHBUFFER_SIZE)
					{
						speechbuffer.erase(speechbuffer.begin(),speechbuffer.begin()+(speechbuffer.size()-MAX_SPEECHBUFFER_SIZE));
					}
				}
			}
		}
		else //add entry mode
		{
			speechbuffer.push_back(s);
		}
	}
	return true;
}
*/
void UserMessage_Callback(const icarus_rover_v2::usermessage::ConstPtr& msg)
{
	icarus_rover_v2::usermessage usermsg;
	usermsg.Level = msg->Level;
	usermsg.message = msg->message;
	diagnostic_status = speakernodeprocess->new_usermessage(usermsg);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
}
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg)
{
	if(msg->Level > NOTICE)
	{
		std::string tempstr;
		switch(msg->Level)
		{
		case NOTICE:
			tempstr += "NOTICE ";
			break;
		case WARN:
			tempstr += "WARN ";
			break;
		case ERROR:
			tempstr += "ERROR ";
			break;
		case FATAL:
			tempstr += "FATAL ";
			break;
		default:
			break;
		}
		std::string tempstr2;
		tempstr2 = msg->Node_Name;
		boost::replace_all(tempstr2,"/"," ");
		boost::replace_all(tempstr2,"_"," ");
		tempstr += tempstr2;
		tempstr += " is ";
		tempstr += msg->Description;
		//speak(tempstr,true);
        icarus_rover_v2::usermessage usermsg;
        usermsg.Level = msg->Level;
        usermsg.message = tempstr;
        //diagnostic_status = speakernodeprocess->new_usermessage(usermsg);
	}
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "speaker_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 8-April-2017";
	fw.Major_Release = SPEAKERNODE_MAJOR_RELEASE;
	fw.Minor_Release = SPEAKERNODE_MINOR_RELEASE;
	fw.Build_Number = SPEAKERNODE_BUILD_NUMBER;
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
	node_name = "speaker_node";
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
	diagnostic_status.Component = COMMUNICATION_NODE;

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
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
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
    device_initialized = false;
    speakernodeprocess = new SpeakerNodeProcess;
    UserMessage_sub = nh.subscribe<icarus_rover_v2::usermessage>("/usermessage",10,UserMessage_Callback);
/*
    sleep(13.0);  //Have to wait for all other nodes to start before doing a mass subscribe
    std::string param_initial_speech_wait = node_name +"/initial_speech_wait";
    if(nh.getParam(param_initial_speech_wait,initial_speech_wait) == false)
    {
    	logger->log_warn("Missing Parameter: initial_speech_wait. Using Default value: 10 Seconds.");
    	initial_speech_wait = 10.0;
    }

    last_time_speech_ended = ros::Time::now();
    */
    sc.reset(new sound_play::SoundClient());
    std::vector<std::string> diagnostic_topics;
    ros::master::V_TopicInfo master_topics;

    ros::master::getTopics(master_topics);
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
    {
    	const ros::master::TopicInfo& info = *it;
    	if(info.datatype == "icarus_rover_v2/diagnostic")
    	{
    		diagnostic_topics.push_back(info.name);
    	}
    }

    ros::Subscriber * diagnostic_subs;
    diagnostic_subs = new ros::Subscriber[diagnostic_topics.size()];

    for(int i = 0; i < diagnostic_topics.size();i++)
    {
    	char tempstr[255];
    	sprintf(tempstr,"i: %d Subscribing to diagnostic topic: %s",i,diagnostic_topics.at(i).c_str());
    	logger->log_info(tempstr);
    	diagnostic_subs[i] = nh.subscribe<icarus_rover_v2::diagnostic>(diagnostic_topics.at(i),1000,diagnostic_Callback);
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
	newdevice.DeviceParent = msg->DeviceParent;
	newdevice.DeviceType = msg->DeviceType;
	newdevice.Capabilities = msg->Capabilities;
	newdevice.ID = msg->ID;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	diagnostic_status = speakernodeprocess->new_devicemsg(newdevice);
	if((device_initialized == false) and (speakernodeprocess->is_finished_initializing() == true))
	{

		resourcemonitor = new ResourceMonitor(diagnostic_status,speakernodeprocess->get_mydevice().Architecture,
																speakernodeprocess->get_mydevice().DeviceName,
																node_name);
		device_initialized = true;
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
