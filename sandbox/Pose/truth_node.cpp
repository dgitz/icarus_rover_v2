#include "truth_node.h"
//Start User Code: Firmware Definition
#define TRUTHNODE_MAJOR_RELEASE 0
#define TRUTHNODE_MINOR_RELEASE 0
#define TRUTHNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
void process_serial_receive()
{
	//printf("starting\n");
	while(kill_node == 0)
	{
		int n = 0;
		int length = 0;
		unsigned char response[256];
		int spot = 0;
		unsigned char buf = '\0';
		memset(response, '\0', sizeof response);
		do
		{
			n = read( serial_device, &buf, 1 );
			length += n;
			//sprintf( &response[spot], "%c", buf );
			response[spot] = buf;
			spot += n;
		} while(buf != '\n' &&  buf != '\r' && n > 0);

		//printf("read: %d\n",length);
		if (n < 0)
		{
			std::cout << "[TruthNode]: Error reading: " << strerror(errno) << std::endl;
		}
		else if (n == 0)
		{
			std::cout << "[TruthNode]: Read nothing!" << std::endl;
		}
		else
		{
			if(length > 4)
			{
				process->new_serialmessage(response,length);
			}
		}


	}
	close(serial_device);
}
bool run_loop1_code()
{

	process->update(1/loop1_rate);
	return true;
}
bool run_loop2_code()
{
	try
	{
		icarus_rover_v2::pose truth_msg;
		bool status = process->get_truthdata(&truth_msg);
		if(status)
		{
			truth_msg.header.stamp = ros::Time::now();
			truth_pub.publish(truth_msg);
			last_truth = truth_msg;
		}
	}
	catch(int e)
	{
		std::cout << "[TruthNode]: An error occurred: truthdata_ready: " << e << std::endl;
	}
	return true;
}
bool run_loop3_code()
{
    
 	return true;
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "truth_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 29-November-2017";
	fw.Major_Release = TRUTHNODE_MAJOR_RELEASE;
	fw.Minor_Release = TRUTHNODE_MINOR_RELEASE;
	fw.Build_Number = TRUTHNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//printf("Delayed rate: %f count: %d\n",process->get_delayrate(),process->get_delayedcounter());
	received_pps = true;
    if((device_initialized == true) and (sensors_initialized == true))
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
    else
    {
    	{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{
					logger->log_error("Got unexpected device message");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
    	}
    	{
    		icarus_rover_v2::srv_device srv;
    		srv.request.query = "DeviceType=Sensor";
    		if(srv_device.call(srv) == true)
    		{
    			for(std::size_t i = 0; i < srv.response.data.size(); i++)
    			{
    				bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
    			}
    		}
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
	node_name = "truth_node";
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
    last_10Hz_timer = ros::Time::now();
    double mtime;
    boost::thread process_serialreceive_thread(&process_serial_receive);
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
            if(run_loop1 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
                if(mtime >= (1.0/loop1_rate))
                {
                    run_loop1_code();
                    last_loop1_timer = ros::Time::now();
                }
            }
            if(run_loop2 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
                if(mtime >= (1.0/loop2_rate))
                {
                    run_loop2_code();
                    last_loop2_timer = ros::Time::now();
                }
            }
            if(run_loop3 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
                if(mtime >= (1.0/loop3_rate))
                {
                    run_loop3_code();
                    last_loop3_timer = ros::Time::now();
                }
            }
            
            mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
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
    process_serialreceive_thread.join();
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
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;

	std::string param_startup_delay = node_name + "/startup_delay";
    double startup_delay = 0.0;
    if(n->getParam(param_startup_delay,startup_delay) == false)
    {
    	logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
    }
    else
    {
    	char tempstr[128];
    	sprintf(tempstr,"Using Parameter: startup_delay = %4.2f sec.",startup_delay);
    	logger->log_notice(std::string(tempstr));
    }
    printf("[%s] Using Parameter: startup_delay = %4.2f sec.\n",node_name.c_str(),startup_delay);
    ros::Duration(startup_delay).sleep();

    std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
    srv_device = nh.serviceClient<icarus_rover_v2::srv_device>(device_topic);

    //std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    //device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
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
        logger->log_warn("Missing parameter: loop1_rate.  Not running loop2 code.");
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
    last_truth.tov = 0.0;
    truth_ready_to_publish = false;

    process = new TruthNodeProcess;
    std::string param_truth_name = node_name + "/devicename_000";
    std::string truth_name;
    if(nh.getParam(param_truth_name,truth_name) == false)
    {
    	logger->log_error("Missing parameter: devicename_000. Exiting.");
    	return false;
    }
    std::string param_truth_id = node_name + "/deviceid_000";
    int truth_id;
    if(nh.getParam(param_truth_id,truth_id) == false)
    {
    	logger->log_error("Missing parameter: deviceid_000. Exiting.");
    	return false;
    }
    //std::string param_truth_pn = node_name + "/devicepn_000";
    std::string truth_pn = "810090";

    /*
    if(nh.getParam(param_truth_pn,truth_pn) == false)
    {
    	logger->log_error("Missing parameter: devicepn_000. Exiting.");
    	return false;
    }
*/
    serialmessagehandler = new SerialMessageHandler;
    process->set_sensorname(truth_pn,truth_name,truth_id);
    process->set_verbositylevel(verbosity_level);
    truth_pub =  nh.advertise<icarus_rover_v2::pose>("/" + truth_name,1);

	diagnostic_status = process->init(diagnostic_status,std::string(hostname));
	sensors_initialized = false;
	usleep(7000000); //Wait for Master Node to initialize

	std::string leverarm_topic = "/" + std::string(hostname) + "_master_node/srv_leverarm";
	srv_leverarm = nh.serviceClient<icarus_rover_v2::srv_leverarm>(leverarm_topic);
	icarus_rover_v2::srv_leverarm srv_la;
	srv_la.request.name = truth_name;
	if(srv_leverarm.call(srv_la) == true)
	{
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"Unable to get LeverArm for: %s from MasterNode.",truth_name.c_str());
		logger->log_error(std::string(tempstr));
		return false;

	}


	std::string connection_topic = "/" + std::string(hostname) + "_master_node/srv_connection";
	srv_connection = nh.serviceClient<icarus_rover_v2::srv_connection>(connection_topic);
	icarus_rover_v2::srv_connection srv_conn;
	srv_conn.request.PartNumber = truth_pn;
	srv_conn.request.ID = truth_id;
	std::string serialdevice_path;
	std::string baudrate;
	if(srv_connection.call(srv_conn) == true)
	{
		serialdevice_path = srv_conn.response.connectionstring.substr(0,srv_conn.response.connectionstring.find(":"));
		baudrate = srv_conn.response.connectionstring.substr(srv_conn.response.connectionstring.find(":")+1);
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"MasterNode Timed out when asked for connection to: %s:%d",truth_pn.c_str(),truth_id);
		logger->log_error(std::string(tempstr));
		return false;
	}

	{
		char tempstr[512];
		sprintf(tempstr,"Connecting %s:%d to %s @ %s (bps)",
				truth_name.c_str(),
				truth_id,
				serialdevice_path.c_str(),
				baudrate.c_str());
		logger->log_notice(std::string(tempstr));
	}
	serial_device = open(serialdevice_path.c_str(),O_RDWR | O_NOCTTY);
	if(serial_device < 0)
	{
		printf("[TruthNode]: Can't open: %s\n",serialdevice_path.c_str());
		return false;
	}
	struct termios tty;
	memset(&tty,0,sizeof tty);
	if(tcgetattr(serial_device,&tty) != 0 )
	{
		std::cout << "[TruthNode]: Error: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}
	if(baudrate == "115200")
	{
		cfsetospeed(&tty,(speed_t)B115200);
		cfsetispeed(&tty,(speed_t)B115200);
	}
	else if(baudrate == "9600")
	{
		cfsetospeed(&tty,(speed_t)B9600);
		cfsetispeed(&tty,(speed_t)B9600);
	}
	else
	{
		printf("[TruthNode]: Baudrate: %s Not Supported.\n",baudrate.c_str());
		return false;
	}
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);
	tcflush( serial_device, TCIFLUSH );
	if ( tcsetattr ( serial_device, TCSANOW, &tty ) != 0) {
		std::cout << "[TruthNode]: Error " << errno << " from tcsetattr" << std::endl;
		return false;
	}

	for(int i = 0; i < 10; i++)
	{//Command normal operation

		unsigned char outbuffer[64];
		int length = 0;

		int tx_status = serialmessagehandler->encode_CommandSerial(outbuffer,&length,ROVERCOMMAND_RUN,ROVERCOMMAND_NONE,ROVERCOMMAND_NONE,ROVERCOMMAND_NONE);
		if(tx_status)
		{
			int count = write( serial_device, outbuffer, length );
			if(count <= 0)
			{
				printf("[TruthNode]: Can't Write to Serial Port. Exiting.\n");
				return false;
			}
			usleep(200000);
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
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{

	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{
			myDevice = device;
			resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
			process->set_mydevice(device);
			device_initialized = true;
		}
	}

	if((device_initialized == true) and (sensors_initialized == false))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
		if(process->get_initialized() == true)
		{
			sensors_initialized = true;
		}
	}
	return true;
}
/*
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
*/
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
