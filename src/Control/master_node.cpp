#include "master_node.h"
//Start Template Code: Firmware Definition
#define MASTERNODE_MAJOR_RELEASE 1
#define MASTERNODE_MINOR_RELEASE 2
#define MASTERNODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
//Start User Code: Functions
double read_device_temperature()
{
	double temp = -100.0;
	ifstream temp_file ("/sys/class/thermal/thermal_zone0/temp");
	std::string line;
	if (temp_file.is_open())
	{
		getline (temp_file,line);
		int t = atoi(line.c_str());
		temp = (double)(t/1000.0); //In Degrees Celcius
		temp = temp*(9.0/5.0) + 32.0;  //To Degrees Farenheit
		temp_file.close();
	}
	return temp;
}
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

	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	return true;
}
bool run_slowrate_code()
{
	publish_deviceinfo();
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
	icarus_rover_v2::resource device_resource_available;
	device_resource_available.Node_Name = myDevice.DeviceName;
	device_resource_available.PID = 0;
	device_resource_available.CPU_Perc = resourcemonitor->get_CPUFree_perc();
	device_resource_available.RAM_MB = (double)(resourcemonitor->get_RAMFree_kB()/1000.0);
	device_resourceavail_pub.publish(device_resource_available);
	if(myDevice.Architecture == "armv7l")
	{
		//diagnostic_status.Diagnostic_Type = SENSORS;
		device_temperature = read_device_temperature();

		if(device_temperature > 130.0)
		{
			diagnostic_status.Diagnostic_Type = SENSORS;
			diagnostic_status.Level = WARN;
			diagnostic_status.Diagnostic_Message = TEMPERATURE_HIGH;
			char tempstr[200];
			sprintf(tempstr,"Device Temperature: %f",device_temperature);
			logger->log_info(tempstr);
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(device_temperature < 50.0)
		{
			diagnostic_status.Diagnostic_Type = SENSORS;
			diagnostic_status.Level = WARN;
			diagnostic_status.Diagnostic_Message = TEMPERATURE_LOW;
			char tempstr[200];
			sprintf(tempstr,"Device Temperature: %f",device_temperature);
			logger->log_info(tempstr);
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
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
	fw.Generic_Node_Name = "master_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 2-Nov-2016";
	fw.Major_Release = MASTERNODE_MAJOR_RELEASE;
	fw.Minor_Release = MASTERNODE_MINOR_RELEASE;
	fw.Build_Number = MASTERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
void publish_deviceinfo()
{
	device_pub.publish(myDevice);
	for(int i = 0; i < otherDevices.size();i++)
	{
		icarus_rover_v2::device other = otherDevices.at(i);
		if(other.DeviceParent == myDevice.DeviceName)
		{
			device_pub.publish(other);
		}
	}

}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
	received_pps = true;
}
//End User Code: Functions
//Start Template Code: Functions
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	logger->log_info("Got command");
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
	//Start Template Code: Initialization and Parameters
	rate = 1;
	verbosity_level = "";
	require_pps_to_start = false;
	received_pps = false;
	char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	myDevice.DeviceName = hostname;
    //Start Template Code: Initialization and Parameters
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;

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
    std::string param_verbosity_level = node_name +"/verbosity_level";

    std::string device_topic = "/" + node_name + "/device";
    device_pub = nh.advertise<icarus_rover_v2::device>(device_topic,1000);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);
	std::string device_resourceavail_topic = "/" + myDevice.DeviceName + "/resource_available";
	device_resourceavail_pub = nh.advertise<icarus_rover_v2::resource>(device_resourceavail_topic,1000);
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
    sleep(3.0);  //Wait for all nodes ot start
    TiXmlDocument device_doc("/home/robot/config/DeviceFile.xml");
    bool devicefile_loaded = device_doc.LoadFile();
    if(devicefile_loaded == true)
    {
    	if(parse_devicefile(device_doc) == true)
    	{
    		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,hostname,node_name);
    	}
    	else { return false; }
    }
    else
    {
    	logger->log_fatal("Could not load or parse /home/robot/config/DeviceFile.xml. Exiting.");
    	return false;
    }
    system("rosnode list -ua > /home/robot/config/AllNodeList");
    string line;
    ifstream allnodelist_file("/home/robot/config/AllNodeList");
    process_file.open("/home/robot/config/ActiveNodes");
   	if(process_file.is_open() == false)
   	{
   		return false;

   	}
    if(allnodelist_file.is_open())
    {
    	while(getline(allnodelist_file,line))
    	{
    		std::vector<std::string> items;
    		boost::split(items,line,boost::is_any_of(":\t"));
    		std::string host = items.at(1).substr(2,items.at(1).size());
    		if(hostname == host)
    		{
    			std::vector<std::string> items2;
    			boost::split(items2,items.at(3),boost::is_any_of("/"));
    			for(int i = 0; i < items2.size();i++)
    			{
    			}
    			std::string node = items2.at(items2.size()-1);
    			if(node != "rosout")
    			{
    				time_t rawtime;
    				struct tm * timeinfo;
    				char datebuffer[80];
    				time (&rawtime);
    				timeinfo = localtime(&rawtime);
    				strftime(datebuffer,80,"%d/%m/%Y %I:%M:%S",timeinfo);
    				process_file << "Node:\t" << node << "\tLaunched:\t" << datebuffer <<  endl;
    			}
    		}
    	}
    	allnodelist_file.close();
    }
    process_file.close();

    device_temperature = -100.0;
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
bool parse_devicefile(TiXmlDocument doc)
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
	        	std::vector<icarus_rover_v2::pin> pins;
				std::vector<std::string> input_topics;
				std::vector<std::string> output_topics;
				std::vector<std::string> topic_modes;
	        	TiXmlElement *l_pDeviceParent = l_pDevice->FirstChildElement( "ParentDevice" );
				if ( NULL != l_pDeviceParent )
				{
					newDevice.DeviceParent = l_pDeviceParent->GetText();
				}

	            TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement( "DeviceName" );
	            if ( NULL != l_pDeviceName )
	            {
	                newDevice.DeviceName = l_pDeviceName->GetText();
	            }

	            TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement( "DeviceType" );
	            if ( NULL != l_pDeviceType )
	            {
	                newDevice.DeviceType = l_pDeviceType->GetText();
	            }

	            TiXmlElement *l_pDeviceArchitecture = l_pDevice->FirstChildElement( "Architecture" );
				if ( NULL != l_pDeviceArchitecture )
				{
					newDevice.Architecture = l_pDeviceArchitecture->GetText();
				}

				TiXmlElement *l_pBoardCount = l_pDevice->FirstChildElement( "BoardCount" );
				if ( NULL != l_pBoardCount )
				{
					newDevice.BoardCount = atoi(l_pBoardCount->GetText());
				}

				TiXmlElement *l_pSensorCount = l_pDevice->FirstChildElement( "SensorCount" );
				if ( NULL != l_pSensorCount )
				{
					newDevice.SensorCount = atoi(l_pSensorCount->GetText());
				}

				TiXmlElement *l_pCapability = l_pDevice->FirstChildElement("Capability");
				std::vector<std::string> capabilities;
				while( l_pCapability )
				{
					std::string capability = l_pCapability->GetText();
					capabilities.push_back(capability);
					l_pCapability = l_pCapability->NextSiblingElement( "Capability" );
				}
				newDevice.Capabilities = capabilities;
				TiXmlElement *l_pPin = l_pDevice->FirstChildElement( "Pin" );
				while( l_pPin )
				{
					icarus_rover_v2::pin newpin;
					TiXmlElement *l_pPinPort = l_pPin->FirstChildElement( "Port" );
					if ( NULL != l_pPinPort )
					{
						newpin.Port = l_pPinPort->GetText();
					}
					TiXmlElement *l_pPinNumber = l_pPin->FirstChildElement( "Number" );
					if ( NULL != l_pPinNumber )
					{
						newpin.Number = atoi(l_pPinNumber->GetText());
					}

					TiXmlElement *l_pPinFunction = l_pPin->FirstChildElement( "Function" );
					if ( NULL != l_pPinFunction )
					{
						newpin.Function = l_pPinFunction->GetText();
					}

					TiXmlElement *l_pPinConnectedDevice = l_pPin->FirstChildElement( "ConnectedDevice" );
					if ( NULL != l_pPinConnectedDevice )
					{
						newpin.ConnectedDevice = l_pPinConnectedDevice->GetText();
					}
					else
					{
						newpin.ConnectedDevice = "";
					}

					l_pPin = l_pPin->NextSiblingElement( "Pin" );
					pins.push_back(newpin);

				}

				newDevice.pins = pins;
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
	return true;
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
std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");

	if(otherDevices.size() < 0)
	{
		status = false;
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables: otherDevices.size() <= 0";
		diaglist.push_back(diag);
	}

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
//End Template Code: Function Definitions
