#include "topicremapper_node.h"
//Start User Code: Firmware Definition
#define TOPICREMAPPERNODE_MAJOR_RELEASE 2
#define TOPICREMAPPERNODE_MINOR_RELEASE 2
#define TOPICREMAPPERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
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
	fw.Description = "Latest Rev: 28-Dec-2016";
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
        TopicMap map = TopicMaps.at(i);
		if(map.in.topic == topic)
		{
			char tempstr[128];
			logger->log_debug(tempstr);
            if(map.in.name == "axis")
            {
                double in_value = msg->axes[map.in.index];
                double out = scale_value(in_value,map.out.neutralvalue,map.in.minvalue,map.in.maxvalue,map.out.minvalue,map.out.maxvalue,map.out.deadband);
                icarus_rover_v2::pin newpin;
                newpin.BoardID = map.out.boardID;
                newpin.ShieldID = map.out.shieldID;
                newpin.DefaultValue = (int)map.out.neutralvalue;
                newpin.Function = map.out.function;
                newpin.Number = map.out.pinnumber;
                newpin.Value = (int)out;
                map.pub.publish(newpin);
            }
            if(map.in.name == "button")
            {
            	icarus_rover_v2::pin newpin;
            	newpin.BoardID = map.out.boardID;
            	newpin.ShieldID = map.out.shieldID;
            	newpin.Function = map.out.function;
            	newpin.Number = map.out.pinnumber;
            	newpin.Value = msg->buttons[map.in.index];
            	map.pub.publish(newpin);
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
                InputChannel in;
                OutputChannel out;
				//Input
				TiXmlElement *l_pInput = l_pTopicMap->FirstChildElement( "InputChannel" );
				if(NULL != l_pInput)
				{
					TiXmlElement *l_pInputType = l_pInput->FirstChildElement( "Type" );
					if(NULL != l_pInputType)
					{
						std::string input_type = l_pInputType->GetText();
						if(input_type == "sensor_msgs/Joy")
						{
							in.type = input_type;
							TiXmlElement *l_pTopic = l_pInput->FirstChildElement( "Topic" );
							if(NULL != l_pTopic)
							{
								in.topic = l_pTopic->GetText();
							}
                            else { return -1; }
							TiXmlElement *l_pName = l_pInput->FirstChildElement( "Name" );
							if(NULL != l_pName)
							{
								in.name = l_pName->GetText();
							}
                            else { return -1; }
                            
                            TiXmlElement *l_pIndex = l_pInput->FirstChildElement("Index");
                            if(NULL != l_pIndex)
                            {
                                in.index = std::atoi(l_pIndex->GetText());
                            }
                            
                            TiXmlElement *l_pMinValue = l_pInput->FirstChildElement("MinValue");
                            if(NULL != l_pMinValue)
                            {
                                in.minvalue = std::atof(l_pMinValue->GetText());
                            }
                            else
                            {
                            	in.minvalue = 0.0;
                            }
                            
                            TiXmlElement *l_pMaxValue = l_pInput->FirstChildElement("MaxValue");
                            if(NULL != l_pMaxValue)
                            {
                                in.maxvalue = std::atof(l_pMaxValue->GetText());
                            }
                            else
                            {
                            	in.maxvalue = 0.0;
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
				TiXmlElement *l_pOutput = l_pTopicMap->FirstChildElement( "OutputChannel" );
				if(NULL != l_pOutput)
				{
					TiXmlElement *l_pOutputType = l_pOutput->FirstChildElement( "Type" );
					if(NULL != l_pOutputType)
					{
						std::string output_type = l_pOutputType->GetText();
						if(output_type == "icarus_rover_v2/pin")
						{
							out.type = output_type;
							TiXmlElement *l_pTopic = l_pOutput->FirstChildElement( "Topic" );
							if(NULL != l_pTopic)
							{
								out.topic = l_pTopic->GetText();
							}
                            else { return -1; }
                            
							TiXmlElement *l_pBoardID = l_pOutput->FirstChildElement( "BoardID" );
							if(NULL != l_pBoardID)
							{
								out.boardID = std::atoi(l_pBoardID->GetText());
							}
                            else { return -1; }
                            
                            TiXmlElement *l_pShieldID = l_pOutput->FirstChildElement( "ShieldID" );
							if(NULL != l_pShieldID)
							{
								out.shieldID = std::atoi(l_pShieldID->GetText());
							}
                            else { return -1; }
                            
                            TiXmlElement *l_pPinNumber = l_pOutput->FirstChildElement( "PinNumber" );
							if(NULL != l_pPinNumber)
							{
								out.pinnumber = std::atoi(l_pPinNumber->GetText());
							}
                            else { return -1; }
                            
                            TiXmlElement *l_pFunction = l_pOutput->FirstChildElement( "Function" );
							if(NULL != l_pFunction)
							{
								out.function = l_pFunction->GetText();
							}
                            else { return -1; }
                            
                            TiXmlElement *l_pMaxValue = l_pOutput->FirstChildElement( "MaxValue" );
							if(NULL != l_pMaxValue)
							{
								out.maxvalue = std::atof(l_pMaxValue->GetText());
							}
                            else { out.maxvalue = 0.0; }
                            
                            TiXmlElement *l_pMinValue = l_pOutput->FirstChildElement( "MinValue" );
							if(NULL != l_pMinValue)
							{
								out.minvalue = std::atof(l_pMinValue->GetText());
							}
                            else { out.minvalue = 0.0; }
                            
                            TiXmlElement *l_pNeutralValue = l_pOutput->FirstChildElement( "NeutralValue" );
							if(NULL != l_pNeutralValue)
							{
								out.neutralvalue = std::atof(l_pNeutralValue->GetText());
							}
                            else { out.neutralvalue = 0.0; }
                            
                            TiXmlElement *l_pDeadband = l_pOutput->FirstChildElement( "Deadband" );
							if(NULL != l_pDeadband)
							{
								out.deadband = std::atof(l_pDeadband->GetText());
							}
                            else { out.deadband = 0.0; }
						}
						else
						{
							char tempstr[128];
							sprintf(tempstr,"Output Topic: %s not supported.  Exiting.",output_type.c_str());
							return 0;
						}
					}
				}
                newtopicmap.in = in;
                newtopicmap.out = out;
				TopicMaps.push_back(newtopicmap);
                
				l_pTopicMap = l_pTopicMap->NextSiblingElement( "TopicMap" );
			}
		}
	}
	return TopicMaps.size();
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
double scale_value(double x,double neutral,double x1,double x2,double y1,double y2, double deadband)
{
    double out = 0.0;
    if(x < (-1.0*deadband))
    {
        double m = (y1-neutral)/(x1-(-1.0*deadband));
        out = m*(x-x1)+y1;    
    }
    else if(x > deadband)
    {
    	double m = (y2-neutral)/(x2-(deadband));
    	out = m*(x-x2)+y2;
    }
    else
    {
        out = neutral;
    }
    return out;
}
//End User Code: Functions
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
            {
                char tempstr[512];
                sprintf(tempstr,"i: %d Input Channel: Type: %s topic: %s name: %s index: %d Min: %f Max: %f",
                    i,
                    TopicMaps.at(i).in.type.c_str(),
                    TopicMaps.at(i).in.topic.c_str(),
                    TopicMaps.at(i).in.name.c_str(),
                    TopicMaps.at(i).in.index,
                    TopicMaps.at(i).in.minvalue,
                    TopicMaps.at(i).in.maxvalue);
                logger->log_debug(tempstr);
            }
            {
                char tempstr[512];
                sprintf(tempstr,"i: %d Output Channel: Type: %s topic: %s BoardID: %d ShieldID: %d Pin: %d Function: %s Max Value: %f Neutral: %f Min Value: %f Deadband: %f",
                    i,
                    TopicMaps.at(i).out.type.c_str(),
                    TopicMaps.at(i).out.topic.c_str(),
                    TopicMaps.at(i).out.boardID,
                    TopicMaps.at(i).out.shieldID,
                    TopicMaps.at(i).out.pinnumber,
                    TopicMaps.at(i).out.function.c_str(),
                    TopicMaps.at(i).out.maxvalue,
                    TopicMaps.at(i).out.neutralvalue,
                    TopicMaps.at(i).out.minvalue,
                    TopicMaps.at(i).out.deadband);
                logger->log_debug(tempstr);
            }
		}
        
		for(int i = 0; i < TopicMaps.size();i++)
		{
			if(TopicMaps.at(i).in.type == "sensor_msgs/Joy")
			{
				ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>(TopicMaps.at(i).in.topic,1000,boost::bind(Joystick_Callback,_1,TopicMaps.at(i).in.topic));
                char tempstr[255];
                sprintf(tempstr,"Subscribing to: %s",TopicMaps.at(i).in.topic.c_str());
                logger->log_info(tempstr);
				TopicMaps.at(i).sub = sub;
            }
			if(TopicMaps.at(i).out.type == "icarus_rover_v2/pin")
			{
				ros::Publisher pub = nh.advertise<icarus_rover_v2::pin>(TopicMaps.at(i).out.topic,1000);
                TopicMaps.at(i).pub = pub;
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
