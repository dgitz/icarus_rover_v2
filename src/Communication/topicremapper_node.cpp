#include "topicremapper_node.h"
//Start Template Code: Firmware Definition
#define TOPICREMAPPERNODE_MAJOR_RELEASE 2
#define TOPICREMAPPERNODE_MINOR_RELEASE 1
#define TOPICREMAPPERNODE_BUILD_NUMBER 2
//End Template Code: Firmware Definition
//Start User Code: Functions
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
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
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "topicremapper_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 30-Nov-2016";
	fw.Major_Release = TOPICREMAPPERNODE_MAJOR_RELEASE;
	fw.Minor_Release = TOPICREMAPPERNODE_MINOR_RELEASE;
	fw.Build_Number = TOPICREMAPPERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
void Joystick_Callback(const sensor_msgs::Joy::ConstPtr& msg,const std::string &topic)
{
	for(int i = 0; i < TopicMaps.size();i++)
	{
		if(TopicMaps.at(i).input_topic_name == topic)
		{
			char tempstr[128];
			sprintf(tempstr,"Got joy from topic: %s",topic.c_str());
			logger->log_debug(tempstr);
			std::vector<std::string> input_strs;
			boost::split(input_strs,TopicMaps.at(i).input_topic_channel,boost::is_any_of(","));
			if(input_strs.at(0) == "axis")
			{
				int index = std::atoi(input_strs.at(1).c_str());
				float value = msg->axes[index];
				if(TopicMaps.at(i).output_topic_type == "icarus_rover_v2/pin")
				{
					icarus_rover_v2::pin output;
					std::vector<std::string> output_strs;
					boost::split(output_strs,TopicMaps.at(i).output_topic_channel,boost::is_any_of(","));
					std::string port = output_strs.at(0);
					int pinnumber = std::atoi(output_strs.at(1).c_str());
					std::string pinmode = output_strs.at(2);
					if(pinmode == "PWMOutput")
					{
						output.Function = pinmode;
						output.Number = pinnumber;
						output.Port = port;
						output.Value = ((255/2.0)*value+127.5);
						TopicMaps.at(i).pub.publish(output);
					}
					else
					{
						char tempstr[128];
						sprintf("Input: %s and Pinmode: %s not currently supported.",input_strs.at(0).c_str(),pinmode.c_str());
						logger->log_warn(tempstr);
					}
				}
			}
			else if(input_strs.at(0) == "button")
			{

				int index = std::atoi(input_strs.at(1).c_str());
				int value = msg->buttons[index];
				if(TopicMaps.at(i).output_topic_type == "icarus_rover_v2/pin")
				{
					icarus_rover_v2::pin output;
					std::vector<std::string> output_strs;
					boost::split(output_strs,TopicMaps.at(i).output_topic_channel,boost::is_any_of(","));
					std::string port = output_strs.at(0);
					int pinnumber = std::atoi(output_strs.at(1).c_str());
					std::string pinmode = output_strs.at(2);
					if((pinmode == "DigitalOutput") || (pinmode == "DigitalOutput-NonActuator"))
					{
						output.Function = pinmode;
						output.Number = pinnumber;
						output.Port = port;
						output.Value = value;
						TopicMaps.at(i).pub.publish(output);
					}
					else
					{
						char tempstr[255];
						sprintf(tempstr,"Input: %s Button: %d and Pinmode: %s not currently supported.",
								input_strs.at(0).c_str(),
								pinnumber,
								pinmode.c_str());
						logger->log_warn(tempstr);
					}
				}
			}
		}
	}

}
int parse_topicmapfile(TiXmlDocument doc)
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
				//Input
				TiXmlElement *l_pInput = l_pTopicMap->FirstChildElement( "InputTopic" );
				if(NULL != l_pInput)
				{
					TiXmlElement *l_pInputType = l_pInput->FirstChildElement( "Type" );
					if(NULL != l_pInputType)
					{
						std::string input_type = l_pInputType->GetText();
						//printf("input: %s\n",input_type.c_str());
						if(input_type == "sensor_msgs/Joy")
						{
							newtopicmap.input_topic_type = input_type;
							TiXmlElement *l_pName = l_pInput->FirstChildElement( "Name" );
							if(NULL != l_pName)
							{
								newtopicmap.input_topic_name = l_pName->GetText();
							}
							TiXmlElement *l_pChannel = l_pInput->FirstChildElement( "Channel" );
							if(NULL != l_pChannel)
							{
								newtopicmap.input_topic_channel = l_pChannel->GetText();
							}
						}
						else
						{
							char tempstr[128];
							sprintf(tempstr,"Input Topic: %s not supported.  Exiting.",input_type.c_str());
							return 0;
						}
					}
				}


				//Output
				TiXmlElement *l_pOutput = l_pTopicMap->FirstChildElement( "OutputTopic" );
				if(NULL != l_pOutput)
				{
					TiXmlElement *l_pOutputType = l_pOutput->FirstChildElement( "Type" );
					if(NULL != l_pOutputType)
					{
						std::string output_type = l_pOutputType->GetText();
						if(output_type == "icarus_rover_v2/pin")
						{
							newtopicmap.output_topic_type = output_type;
							TiXmlElement *l_pName = l_pOutput->FirstChildElement( "Name" );
							if(NULL != l_pName)
							{
								newtopicmap.output_topic_name = l_pName->GetText();
							}
							TiXmlElement *l_pChannel = l_pOutput->FirstChildElement( "Channel" );
							if(NULL != l_pChannel)
							{
								newtopicmap.output_topic_channel = l_pChannel->GetText();
							}
						}
						else
						{
							char tempstr[128];
							sprintf(tempstr,"Output Topic: %s not supported.  Exiting.",output_type.c_str());
							return 0;
						}
					}
				}
				TopicMaps.push_back(newtopicmap);
				l_pTopicMap = l_pTopicMap->NextSiblingElement( "TopicMap" );
			}
		}
	}
	return TopicMaps.size();
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
 
	node_name = "topicremapper_node";


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
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		ros::spinOnce();
		loop_rate.sleep();
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
    //Start Template Code: Initialization and Parameters
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
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
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
    //joy_sub = nh.subscribe<sensor_msgs::Joy>("/Diagnostics_GUI/joystick",1000,Joystick_Callback);

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
		if(parse_topicmapfile(topicmap_doc) <= 0)
		{
			logger->log_fatal("Unable to parse TopicMap.xml.  Exiting.");
			return false;
		}
		for(int i = 0; i < TopicMaps.size();i++)
		{
			char tempstr[256];
			sprintf(tempstr,"i: %d in: %s\n%s\n%s out: %s\n%s\n%s",i,
					TopicMaps.at(i).input_topic_type.c_str(),TopicMaps.at(i).input_topic_name.c_str(),TopicMaps.at(i).input_topic_channel.c_str(),
					TopicMaps.at(i).output_topic_type.c_str(),TopicMaps.at(i).output_topic_name.c_str(),TopicMaps.at(i).output_topic_channel.c_str());
			logger->log_debug(tempstr);
		}
		for(int i = 0; i < TopicMaps.size();i++)
		{
			if(TopicMaps.at(i).input_topic_type == "sensor_msgs/Joy")
			{
				char tempstr[256];
				sprintf(tempstr,"Subscribing to: %s\n%s\n%s\n out: %s\n%s\n%s\n",
						TopicMaps.at(i).input_topic_type.c_str(),TopicMaps.at(i).input_topic_name.c_str(),TopicMaps.at(i).input_topic_channel.c_str(),
						TopicMaps.at(i).output_topic_type.c_str(),TopicMaps.at(i).output_topic_name.c_str(),TopicMaps.at(i).output_topic_channel.c_str());
				logger->log_debug(tempstr);
				//ros::Subscriber joy_sub;
				joy_sub = nh.subscribe<sensor_msgs::Joy>(TopicMaps.at(i).input_topic_name,1000,boost::bind(Joystick_Callback,_1,TopicMaps.at(i).input_topic_name));
			}
			if(TopicMaps.at(i).output_topic_type == "icarus_rover_v2/pin")
			{
				TopicMaps.at(i).pub = nh.advertise<icarus_rover_v2::pin>(TopicMaps.at(i).output_topic_name,1000);
			}
		}
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
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
	if((device_initialized == false) and newdevice.DeviceName == hostname)
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
