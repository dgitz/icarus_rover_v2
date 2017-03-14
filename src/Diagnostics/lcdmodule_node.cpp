#include "lcdmodule_node.h"
//Start User Code: Firmware Definition
#define LCDMODULENODE_MAJOR_RELEASE 0
#define LCDMODULENODE_MINOR_RELEASE 0
#define LCDMODULENODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg)
{
	char tempstr_top[16];
	char tempstr_bottom[16];


	switch(msg->Level)
	{
		case DEBUG:
			//logger->log_debug(tempstr);
			break;
		case INFO:
			sprintf(tempstr_top,"INFO:");
			sprintf(tempstr_bottom,"NORMAL");
			break;
		case NOTICE:
			//logger->log_notice(tempstr);
			break;
		case WARN:
			//logger->log_warn(tempstr);
			break;
		case ERROR:
			//logger->log_error(tempstr);
			break;
		case FATAL:
			//logger->log_fatal(tempstr);
			break;
		default:
			break;
	}
	write(lcdDevice.device_fid,(const void *)0xFF,1);
	sleep(1);
	write(lcdDevice.device_fid,(const void *)0x80,1);
	sleep(1);
	write(lcdDevice.device_fid,tempstr_top,5);
	sleep(1);
	write(lcdDevice.device_fid,(const void *)0xFF,1);
	sleep(1);
	write(lcdDevice.device_fid,(const void *)0xBB,1);
	sleep(1);
	write(lcdDevice.device_fid,tempstr_bottom,6);
	sleep(1);
}
bool run_fastrate_code()
{
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
	diagnostic_pub.publish(diagnostic_status);
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "lcdmodule_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 10-March-2017";
	fw.Major_Release = LCDMODULENODE_MAJOR_RELEASE;
	fw.Minor_Release = LCDMODULENODE_MINOR_RELEASE;
	fw.Build_Number = LCDMODULENODE_BUILD_NUMBER;
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
	node_name = "diagnostic_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();

    if(initializenode() == false)
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
    close(lcdDevice.device_fid);
    logger->log_notice("Node Finished Safely.");
    return 0;
}
bool initializenode()
{
    //Start Template Code: Initialization, Parameters and Topics
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,1000);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("FATAL",ros::this_node::getName());
		logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}
	std::string param_loop_rate = node_name +"/loop_rate";
	if(n->getParam(param_loop_rate,rate) == false)
	{
		logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
		return false;
	}
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;
	std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	device_sub = n->subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	pps_sub = n->subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
 	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
	//End Template Code: Initialization, Parameters and Topics

	//Start User Code: Initialization, Parameters and Topics
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = DIAGNOSTIC_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	lcdDevice.location = "/dev/ttyUSB0";
	int device_fid = open(lcdDevice.location.c_str(),O_RDWR | O_NOCTTY);
	if(device_fid < 0)
	{
		char tempstr[255];
		sprintf(tempstr,"Unable to setup LCD at: %s Exiting.",lcdDevice.location.c_str());
		logger->log_fatal(tempstr);
		return false;
	}
	struct termios options;
	tcgetattr(device_fid, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;

	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 50;
	cfmakeraw(&options);
	tcflush(device_fid, TCIFLUSH);
	tcsetattr(device_fid, TCSANOW, &options);

	int iFlags;
	iFlags = TIOCM_RTS;
	ioctl(device_fid, TIOCMBIS, &iFlags);
	sleep(1.0);
	// turn off RTS
	iFlags = TIOCM_RTS;
	ioctl(device_fid, TIOCMBIC, &iFlags);

	// turn on DTR
	iFlags = TIOCM_DTR;
	ioctl(device_fid, TIOCMBIS, &iFlags);
	sleep(1.0);
	// turn off DTR
	iFlags = TIOCM_DTR;
	ioctl(device_fid, TIOCMBIC, &iFlags);

	lcdDevice.device_fid = device_fid;
	ros::Subscriber sub = n->subscribe<icarus_rover_v2::diagnostic>("/dgitzrosmaster_diagnostic_node/diagnostic",1000,diagnostic_Callback);
	diag_subs.push_back(sub);
	//End User Code: Initialization, Parameters and Topics
    logger->log_info("Initialized!");
    return true;
}
//End Initialize Function

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

	if((newdevice.DeviceName == hostname) && (device_initialized == false))
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
//End Template Code: Functions
