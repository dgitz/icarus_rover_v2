#include "speaker_node.h"
//Start Template Code: Firmware Definition
#define SPEAKERNODE_MAJOR_RELEASE 1
#define SPEAKERNODE_MINOR_RELEASE 2
#define SPEAKERNODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
//Start User Code: Functions
bool speak(std::string s,bool mode)
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
	return true;
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
		speak(tempstr,true);

	}
}
void device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
}

bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	speak("",false);
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
		bool status = resourcemonitor->update();
		if(status == true)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
		else
		{
			logger->log_warn("Couldn't read resources used.");
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
	fw.Generic_Node_Name = "speaker_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 12-Oct-2016";
	fw.Major_Release = SPEAKERNODE_MAJOR_RELEASE;
	fw.Minor_Release = SPEAKERNODE_MINOR_RELEASE;
	fw.Build_Number = SPEAKERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
//End User Code: Functions

//Start Template Code: Functions
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
        return 0; 
    }
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;

    while (ros::ok())
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
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
	sleep(5);
    //Start Template Code: Initialization and Parameters
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
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
    std::string param_loop_rate = node_name +"/loop_rate";
    if(nh.getParam(param_loop_rate,rate) == false)
    {
        logger->log_warn("Missing Parameter: loop_rate.");
        return false;
    }
    char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023);
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
    last_time_speech_ended = ros::Time::now();
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
    	char tempstr[50];
    	sprintf(tempstr,"Subscribing to diagnostic topic: %s",diagnostic_topics.at(i).c_str());
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
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	myDevice = newdevice;
	if(myDevice.DeviceName != "")
	{
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
//End Template Code: Functions
