#include "topicmapper_node.h"
//Start Template Code: Firmware Definition
#define TOPICMAPPERNODE_MAJOR_RELEASE 1
#define TOPICMAPPERNODE_MINOR_RELEASE 1
#define TOPICMAPPERNODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
//Start User Code: Functions
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
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
	fw.Generic_Node_Name = "topicmapper_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 16-Sep-2016";
	fw.Major_Release = TOPICMAPPERNODE_MAJOR_RELEASE;
	fw.Minor_Release = TOPICMAPPERNODE_MINOR_RELEASE;
	fw.Build_Number = TOPICMAPPERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
void parse_topicmapfile(TiXmlDocument doc)
{
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
		// set of &lt;person&gt; tags
		TiXmlElement *l_pTopicMapList = l_pRootElement->FirstChildElement( "TopicMapList" );

		if ( NULL != l_pTopicMapList )
		{
			TiXmlElement *l_pTopicMap = l_pTopicMapList->FirstChildElement( "TopicMap" );

			while( l_pTopicMap )
			{
				TopicMap newtopicmap;
				TiXmlElement *l_pInputTopic = l_pTopicMap->FirstChildElement( "InputTopic" );
				if ( NULL != l_pInputTopic )
				{
					newtopicmap.input_topic = l_pInputTopic->GetText();
				}
				TiXmlElement *l_pOutputTopic = l_pTopicMap->FirstChildElement( "OutputTopic" );
				if ( NULL != l_pOutputTopic )
				{
					newtopicmap.output_topic = l_pOutputTopic->GetText();
				}
				TiXmlElement *l_pMode = l_pTopicMap->FirstChildElement( "Mode" );
				if ( NULL != l_pMode )
				{
					newtopicmap.topic_mode = l_pMode->GetText();
				}

				TopicMaps.push_back(newtopicmap);
				l_pTopicMap = l_pTopicMap->NextSiblingElement( "TopicMap" );
			}
		}
	}
/*
 TiXmlElement *l_pTopicMap = l_pDevice->FirstChildElement("TopicMap");
				while(l_pTopicMap)
				{
					std::string input_topic;
					std::string output_topic;
					std::string topic_mode;
					TiXmlElement *l_pInputTopic = l_pTopicMap->FirstChildElement( "InputTopic" );
					if ( NULL != l_pInputTopic )
					{
						input_topic = l_pInputTopic->GetText();
					}
					TiXmlElement *l_pOutputTopic = l_pTopicMap->FirstChildElement( "OutputTopic" );
					if ( NULL != l_pOutputTopic )
					{
						output_topic = l_pOutputTopic->GetText();
					}
					TiXmlElement *l_pTopicMode = l_pTopicMap->FirstChildElement( "Mode" );
					if ( NULL != l_pTopicMode )
					{
						topic_mode = l_pTopicMode->GetText();
					}
					input_topics.push_back(input_topic);
					output_topics.push_back(output_topic);
					topic_modes.push_back(topic_mode);
					l_pTopicMap = l_pTopicMap->NextSiblingElement("TopicMap");
				}
				newDevice.InputTopics = input_topics;
				newDevice.OutputTopics = output_topics;
				newDevice.TopicModes = topic_modes;
 */
}
//End User Code: Functions

//Start Template Code: Functions
int main(int argc, char **argv)
{
 
	node_name = "topicmapper_node";


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
    //Start Template Code: Initialization and Parameters
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
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
    TiXmlDocument topicmap_doc("/home/robot/config/TopicMap.xml");
	bool topicmapfile_loaded = topicmap_doc.LoadFile();
	if(topicmapfile_loaded == true)
	{
		parse_topicmapfile(topicmap_doc);
		for(int i = 0; i < TopicMaps.size();i++)
		{
			char tempstr[256];
			sprintf(tempstr,"i: %d in: %s out: %s mode: %s",i,TopicMaps.at(i).input_topic.c_str(),
													  TopicMaps.at(i).output_topic.c_str(),
													  TopicMaps.at(i).topic_mode.c_str());
			logger->log_debug(tempstr);
		}
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,hostname,node_name);
	}
	else
	{
		logger->log_fatal("Could not load or parse /home/robot/config/TopicMap.xml. Exiting.");
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
	newdevice.DeviceType = msg->DeviceType;
	newdevice.DeviceParent = msg->DeviceParent;
	if((device_initialized == false))
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
//End Template Code: Functions
