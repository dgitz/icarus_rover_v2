#include "master_node.h"

bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	return true;
}
bool run_slowrate_code()
{
	publish_deviceinfo();
	//logger->log_debug("Running slow rate code.");
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = DEBUG;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");

	return true;
}
void publish_deviceinfo()
{
	device_pub.publish(myDevice);
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
	received_pps = true;
}
//End Template Code: Function Definitions

//Start User Code: Function Definitions
//Finish User Code: Function Definitions
int main(int argc, char **argv)
{
 
	node_name = "master_node";


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
	rate = 1;
	verbosity_level = "";
	require_pps_to_start = false;
	received_pps = false;
	char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	myDevice.DeviceName = hostname;
    //Start Template Code: Initialization and Parameters
    //printf("Node name: %s",node_name.c_str());
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = CONTROLLER_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);
    std::string param_verbosity_level = node_name +"/verbosity_level";

    std::string device_topic = "/" + node_name + "/device";
    device_pub = nh.advertise<icarus_rover_v2::device>(device_topic,1000);

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
    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}


    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    TiXmlDocument device_doc("/home/robot/config/DeviceFile.xml");
    bool devicefile_loaded = device_doc.LoadFile();
    if(devicefile_loaded == true)
    {
    	parse_devicefile(device_doc);
    }
    else
    {
    	logger->log_fatal("Could not load or parse /home/robot/config/DeviceFile.xml. Exiting.");
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
    //print_myDevice();
    //print_otherDevices();
    return true;
    //End Template Code: Finish Initialization.
}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void parse_devicefile(TiXmlDocument doc)
{
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
	    // set of &lt;person&gt; tags
	    TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement( "DeviceList" );

	    if ( NULL != l_pDeviceList )
	    {
	        TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement( "Device" );

	        while( l_pDevice )
	        {
	        	icarus_rover_v2::device newDevice;
	            TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement( "DeviceName" );

	            if ( NULL != l_pDeviceName )
	            {
	                newDevice.DeviceName = l_pDeviceName->GetText();
	            }

	            TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement( "DeviceType" );

	            if ( NULL != l_pDeviceType )
	            {
	                //std::cout << " " << l_pDeviceType->GetText();
	            }

	            TiXmlElement *l_pDeviceArchitecture = l_pDevice->FirstChildElement( "Architecture" );

				if ( NULL != l_pDeviceArchitecture )
				{
					//std::cout << " " << l_pDeviceArchitecture->GetText();
					newDevice.Architecture = l_pDeviceArchitecture->GetText();
				}

	            //std::cout << std::endl;
	            if(newDevice.DeviceName == myDevice.DeviceName)
	            {
	            	//This is me.
	            	myDevice = newDevice;
	            }
	            else
	            {
	            	otherDevices.push_back(newDevice);
	            }
	            l_pDevice = l_pDevice->NextSiblingElement( "Device" );
	        }
	    }
	}
}
void print_myDevice()
{
	printf("MY DEVICE\r\n------------------------\r\n");
	printf("Device Name: %s\r\n",myDevice.DeviceName.c_str());
	printf("Architecture: %s\r\n",myDevice.Architecture.c_str());

	printf("----------------------\r\n");
}
void print_otherDevices()
{
	printf("OTHER DEVICES\r\n------------------------\r\n");
	for(int i = 0; i < otherDevices.size();i++)
	{
		printf("Device Name: %s\r\n",otherDevices.at(i).DeviceName.c_str());
		printf("Architecture: %s\r\n",otherDevices.at(i).Architecture.c_str());
	}

	printf("----------------------\r\n");
}
//End Template Code: Function Definitions
