OBSOLETE!!!
#include "batterymonitor_node.h"
//Start User Code: Firmware Definition
#define BATTERYMONITORNODE_MAJOR_RELEASE 0
#define BATTERYMONITORNODE_MINOR_RELEASE 0
#define BATTERYMONITORNODE_BUILD_NUMBER 2
//End User Code: Firmware Definition
//Start User Code: Functions
void cellvoltage_Callback(const std_msgs::Float32::ConstPtr& msg,const std::string &topicname)
{
	printf("Topic: %s Voltage: %f\n",topicname.c_str(),msg->data);
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "batterymonitor_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 8-April-2017";
	fw.Major_Release = BATTERYMONITORNODE_MAJOR_RELEASE;
	fw.Minor_Release = BATTERYMONITORNODE_MINOR_RELEASE;
	fw.Build_Number = BATTERYMONITORNODE_BUILD_NUMBER;
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
void PPS10_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
}
void PPS100_Callback(const std_msgs::Bool::ConstPtr& msg)
{
   	//logger->log_debug("Running fast rate code.");
	//diagnostic_status = process->update(.01);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
}
void PPS1000_Callback(const std_msgs::Bool::ConstPtr& msg)
{
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

int main(int argc, char **argv)
{
	node_name = "batterymonitor_node";
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
    ros::Rate loop_rate(1);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
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
	diagnostic_status.Component = POWER_NODE;

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
    pps10_sub = nh.subscribe<std_msgs::Bool>("/10PPS",1000,PPS10_Callback); 
    pps100_sub = nh.subscribe<std_msgs::Bool>("/100PPS",1000,PPS100_Callback); 
    pps1000_sub = nh.subscribe<std_msgs::Bool>("/1000PPS",1000,PPS1000_Callback); 
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
    bool search_for_batteries = true;
    int batteryindex = 1;
    while(search_for_batteries == true)
    {
        std::string batteryname;
        std::string param_batteryname = node_name +"/battery" + boost::lexical_cast<std::string>(batteryindex);
        if(nh.getParam(param_batteryname,batteryname) == false)
        {
            char tempstr[255];
            sprintf(tempstr,"Didn't find battery at: %s Not adding anymore.",param_batteryname.c_str());
            logger->log_info(tempstr);
            search_for_batteries = false;
        }
        else
        {
            Battery battery;
            battery.name = batteryname;
            batteries.push_back(battery);
        }
        batteryindex++;
    }
    
    TiXmlDocument device_doc("/home/robot/config/DeviceFile.xml");
    bool devicefile_loaded = device_doc.LoadFile();
    if(devicefile_loaded == true)
    {
    	if(parse_devicefile(device_doc) == true)
    	{
    		
    	}
    	else { return false; }
    }
    else
    {
    	logger->log_fatal("Could not load or parse /home/robot/config/DeviceFile.xml. Exiting.");
    	return false;
    }

    for(int b = 0; b < batteries.size(); b++)
    {
    	for(int c = 0; c < batteries.at(b).cells.size(); c++)
    	{
    		ros::Subscriber cell_sub = nh.subscribe<std_msgs::Float32>(batteries.at(b).cells.at(c).voltage_topic,
    				1000,boost::bind(cellvoltage_Callback,_1,batteries.at(b).cells.at(c).voltage_topic));
    		cellvoltage_subs.push_back(cell_sub);
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
bool parse_devicefile(TiXmlDocument doc)
{
    for(int i = 0; i < batteries.size(); i++)
    {
        batteries.at(i).initialized = false;
    }
    for(int i = 0; i < batteries.size(); i++)
    {
        TiXmlElement *l_pRootElement = doc.RootElement();

        if( NULL != l_pRootElement )
        {
            TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement( "DeviceList" );

            if ( NULL != l_pDeviceList )
            {
                TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement( "Device" );

                while( l_pDevice )
                {
                    

                    TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement( "DeviceName" );
                    if ( NULL != l_pDeviceName )
                    {
                        if(l_pDeviceName->GetText() == batteries.at(i).name)
                        {
                            int series_count,parallel_count;
                            TiXmlElement *l_pSeries = l_pDevice->FirstChildElement( "Series" );
                            if ( NULL != l_pSeries )
                            {
                                series_count = atoi(l_pSeries->GetText());
                            }

                            TiXmlElement *l_pParallel = l_pDevice->FirstChildElement( "Parallel" );
                            if ( NULL != l_pParallel )
                            {
                                parallel_count = atoi(l_pParallel->GetText());
                            }
                            
                            TiXmlElement *l_pVoltage = l_pDevice->FirstChildElement( "Voltage" );
                            if ( NULL != l_pVoltage )
                            {
                                batteries.at(i).rated_voltage = atof(l_pVoltage->GetText());
                            }
                            
                            TiXmlElement *l_pCapacity = l_pDevice->FirstChildElement( "Capacity" );
                            if ( NULL != l_pCapacity )
                            {
                                batteries.at(i).capacity_Ah = atof(l_pCapacity->GetText());
                            }
                            
                            TiXmlElement *l_pChemistry = l_pDevice->FirstChildElement( "Chemistry" );
                            if ( NULL != l_pChemistry )
                            {
                                batteries.at(i).chemistry = l_pChemistry->GetText();
                            }
                            
                            int total_cell_count = series_count*parallel_count;
                            TiXmlElement *l_pCell = l_pDevice->FirstChildElement("Cell");
                            int i = 0;
                            while(l_pCell)
                            {
                            	Cell cell;
                            	TiXmlElement *l_pCellName = l_pCell->FirstChildElement( "Name" );
                            	if ( NULL != l_pCellName )
                            	{
                            		cell.name = l_pCellName->GetText();
                            	}

                            	TiXmlElement *l_pCellVoltage = l_pCell->FirstChildElement( "Voltage" );
                            	if ( NULL != l_pCellVoltage )
                            	{
                            		cell.voltage = atof(l_pCellVoltage->GetText());
                            	}

                            	cell.voltage_topic = "/" + batteries.at(i).name + "/" + cell.name + "/voltage";
                            	batteries.at(i).cells.push_back(cell);
                            	l_pCell = l_pCell->NextSiblingElement("Cell");

                            }
                            if(batteries.at(i).cells.size() != total_cell_count)
                            {
                            	char tempstr[512];
                            	sprintf(tempstr,"Looking for %d Cells in Battery: %s but only found: %d",
                            			total_cell_count,batteries.at(i).name.c_str(),batteries.at(i).cells.size());
                            	logger->log_fatal(std::string(tempstr));
                            }
                        }
                        
                    }
                    l_pDevice = l_pDevice->NextSiblingElement( "Device" );
                }
            }
        }
    }
	return true;
}
