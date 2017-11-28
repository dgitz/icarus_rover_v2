#include "master_node.h"
//Start User Code: Firmware Definition
#define MASTERNODE_MAJOR_RELEASE 1
#define MASTERNODE_MINOR_RELEASE 4
#define MASTERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
//Start User Code: Function Prototypes
bool check_serialports()
{

	std::vector<MasterNodeProcess::SerialPort> ports = process.get_serialports();
	std::vector<std::string> baudrates = process.get_allserialbaudrates();
	for(std::size_t i = 0; i < ports.size(); i++)
	{
		bool port_valid = false;
		bool continue_checking_port = true;
		for(std::size_t j = 0; j < baudrates.size(); j++)
		{
			if(continue_checking_port == true)
			{
				char tempstr[512];
				sprintf(tempstr,"Checking Serial Port: %s @ %s bps\n",ports.at(i).file.c_str(),baudrates.at(j).c_str());
				logger->log_info(std::string(tempstr));
				std::string baudrate = baudrates.at(j);
				int dev_fd = open(ports.at(i).file.c_str(),O_RDWR | O_NOCTTY);
				if(dev_fd < 0)
				{
					printf("[MasterNode]: Can't open: %s\n",ports.at(i).file.c_str());
					break;
				}
				struct termios tty;
				memset(&tty,0,sizeof tty);
				if(tcgetattr(dev_fd,&tty) != 0 )
				{
					std::cout << "[MasterNode]: Error: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
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
					printf("[MasterNode]: Baudrate: %s Not Supported.\n",baudrate.c_str());
				}
				tty.c_cflag     &=  ~PARENB;            // Make 8n1
				tty.c_cflag     &=  ~CSTOPB;
				tty.c_cflag     &=  ~CSIZE;
				tty.c_cflag     |=  CS8;

				tty.c_cflag     &=  ~CRTSCTS;           // no flow control
				tty.c_cc[VMIN]   =  1;                  // read doesn't block
				tty.c_cc[VTIME]  =  50;                  // 0.5 seconds read timeout
				tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

				/* Make raw */
				cfmakeraw(&tty);
				tcflush( dev_fd, TCIFLUSH );
				if ( tcsetattr ( dev_fd, TCSANOW, &tty ) != 0) {
					std::cout << "[MasterNode]: Error " << errno << " from tcsetattr" << std::endl;
				}

				{//Command a reset
					unsigned char outbuffer[64];
					int length = 0;

					int tx_status = serialmessagehandler->encode_CommandSerial(outbuffer,&length,ROVERCOMMAND_BOOT,ROVERCOMMAND_NONE,ROVERCOMMAND_NONE,ROVERCOMMAND_NONE);
					if(tx_status)
					{
						int count = write( dev_fd, outbuffer, length );
						if(count <= 0)
						{
							break;
						}
						usleep(2000000);
					}

				}
				int packets_to_check_counter = 0;
				while((packets_to_check_counter < 10) && (port_valid != true))
				{
					int n = 0;
					int length = 0;
					unsigned char response[64];
					int spot = 0;
					unsigned char buf = '\0';
					memset(response, '\0', sizeof response);
					do
					{
						n = read( dev_fd, &buf, 1 );
						//printf("%c",buf);
						length += n;
						//sprintf( &response[spot], "%c", buf );
						response[spot] = buf;
						spot += n;
					} while(buf != '\n' &&  buf != '\r' && n > 0);
					if (n < 0)
					{
						std::cout << "Error reading: " << strerror(errno) << std::endl;
					}
					else if (n == 0)
					{
						std::cout << "Read nothing!" << std::endl;
					}
					else
					{
						if(length > 4)
						{
							bool status = process.new_serialmessage(ports.at(i).file,baudrate,response,length);
							if(status == true)
							{
								continue_checking_port = false;
								port_valid = true;
							}
							else
							{
								usleep(200000);
							}

						}
					}
					packets_to_check_counter++;
				}
				close(dev_fd);
			}
		}
	}
	ports = process.get_serialports();
	for(std::size_t i = 0; i < ports.size(); i++)
	{
		if(ports.at(i).available == true)
		{
			char tempstr[512];
			sprintf(tempstr,"Found ROS Device on Serial Port: %s @ %s with PN=%s ID=%d\n",
					ports.at(i).file.c_str(),
					ports.at(i).baudrate.c_str(),
					ports.at(i).pn.c_str(),
					ports.at(i).id);
			logger->log_notice(std::string(tempstr));
		}
		else
		{
			char tempstr[512];
			sprintf(tempstr,"No ROS Device found on: %s\n",
					ports.at(i).file.c_str());
			logger->log_warn(std::string(tempstr));
		}
	}
	return true;
}
std::vector<std::string> find_serialports()
{
    std::vector<std::string> ports;
    std::vector<std::string> files;
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir("/dev/")) == NULL) 
    {
        cout << "Error(" << errno << ") opening " << "/dev/" << endl;
        //return errno;
    }

    while ((dirp = readdir(dp)) != NULL) 
    {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    std::string serial_usb = "ttyUSB";
    std::string serial_acm = "ttyACM";
   // std::string serial = "ttyS"; //Get rid of this one
    for(std::size_t i = 0; i < files.size(); i++)
    {
        std::size_t found_usb = files.at(i).find(serial_usb);
        std::size_t found_acm = files.at(i).find(serial_acm);
        //std::size_t found_serial = files.at(i).find(serial);
        if( (found_usb != std::string::npos) || 
            (found_acm != std::string::npos))
            //(found_serial != std::string::npos))
        {
            ports.push_back(files.at(i));
        }
    }
    return ports;
}
bool connection_service(icarus_rover_v2::srv_connection::Request &req,
					icarus_rover_v2::srv_connection::Response &res)
{
	std::vector<MasterNodeProcess::SerialPort> ports = process.get_serialports();
	for(std::size_t i = 0; i < ports.size(); i++)
	{
		if((ports.at(i).pn == req.PartNumber) and (ports.at(i).id == req.ID))
		{
			res.connectionstring = ports.at(i).file + ":" + ports.at(i).baudrate;
			return true;
		}
	}
	return false;
}
bool device_service(icarus_rover_v2::srv_device::Request &req,
				icarus_rover_v2::srv_device::Response &res)
{
	if(req.query == "SELF")
	{
		res.data.push_back(process.get_mydevice());
		return true;
	}
	else if(std::string::npos != req.query.find("DeviceType="))
	{
		std::string devicetype = req.query.substr(11,req.query.size());
		for(std::size_t i = 0; i < devices_to_publish.size(); i++)
		{
			if(devices_to_publish.at(i).DeviceType == devicetype)
			{
				res.data.push_back(devices_to_publish.at(i));
			}
		}
		return true;
	}
	return false;
}
bool leverarm_service(icarus_rover_v2::srv_leverarm::Request &req,
				icarus_rover_v2::srv_leverarm::Response &res)
{
	icarus_rover_v2::leverarm la;
	bool status = process.get_leverarm(&la,req.name);
	if(status == false) { return false; }
	res.lever = la;
	return true;
}
bool run_loop1_code()
{
    publish_deviceinfo();
	icarus_rover_v2::resource device_resource_available;
	device_resource_available.Node_Name = process.get_mydevice().DeviceName;
	device_resource_available.PID = 0;
	device_resource_available.CPU_Perc = resourcemonitor->get_CPUFree_perc();
	device_resource_available.RAM_MB = (double)(resourcemonitor->get_RAMFree_kB()/1000.0);
	device_resourceavail_pub.publish(device_resource_available);
	if(process.get_mydevice().Architecture == "armv7l")
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
	else
	{
		logger->log_error("Unable to read system temperature.");
	}
	return temp;
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "master_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 27-May-2017";
	fw.Major_Release = MASTERNODE_MAJOR_RELEASE;
	fw.Minor_Release = MASTERNODE_MINOR_RELEASE;
	fw.Build_Number = MASTERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    received_pps = true;
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
void publish_deviceinfo()
{
    //ros::Time start = ros::Time::now();
	device_pub.publish(process.get_mydevice());
    for(int i = 0; i < devices_to_publish.size(); i++)
    {
        device_pub.publish(devices_to_publish.at(i));
    }
    //printf("Time to publish: %f\n",measure_time_diff(ros::Time::now(),start));
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
    device_initialized = false;
    hostname[1023] = '\0';
    gethostname(hostname,1023);
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
    diagnostic_status = process.init(diagnostic_status,std::string(hostname));
    diagnostic_status = process.load_devicefile("/home/robot/config/DeviceFile.xml");
    if(diagnostic_status.Level >= WARN)
    {
        logger->log_diagnostic(diagnostic_status);
        printf("[MasterNode] ERROR: %s\n",diagnostic_status.Description.c_str());
        return false;
    }
    diagnostic_status = process.load_systemfile("/home/robot/config/SystemFile.xml");
    if(diagnostic_status.Level >= WARN)
    {
    	logger->log_diagnostic(diagnostic_status);
    	printf("[MasterNode] ERROR: %s\n",diagnostic_status.Description.c_str());
    	return false;
    }
    

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);
	std::string device_resourceavail_topic = "/" + process.get_mydevice().DeviceName + "/resource_available";
	device_resourceavail_pub = nh.advertise<icarus_rover_v2::resource>(device_resourceavail_topic,1000);

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
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
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
    serialmessagehandler = new SerialMessageHandler;
    resourcemonitor = new ResourceMonitor(diagnostic_status,process.get_mydevice().Architecture,hostname,node_name);
    if(resourcemonitor == NULL)
    {
        logger->log_error("Couldn't initialize resourcemonitor. Exiting.\n");
        printf("[MasterNode]: Couldn't initialize resourcemonitor. Exiting.\n");
        return false;
    }
    system("rosnode list -ua > /home/robot/config/AllNodeList");
    
    bool update_nodelist = process.update_nodelist("/home/robot/config/AllNodeList","/home/robot/config/ActiveNodes");
    if(update_nodelist == false)
    {
        logger->log_error("Couldn't initialize ActiveNodeList. Exiting.\n");
        printf("[MasterNode]: Couldn't initialize ActiveNodeList. Exiting.\n");
        return false;
    }
    device_temperature = -100.0;
    devices_to_publish = process.get_childdevices();

    std::string srv_device_topic = "/" + node_name + "/srv_device";
    device_srv = nh.advertiseService(srv_device_topic,device_service);
    std::string srv_connection_topic = "/" + node_name + "/srv_connection";
    connection_srv = nh.advertiseService(srv_connection_topic,connection_service);
    std::string srv_leverarm_topic = "/" + node_name + "/srv_leverarm";
    leverarm_srv = nh.advertiseService(srv_leverarm_topic,leverarm_service);
    
    diagnostic_status = process.set_serialportlist(find_serialports());

    if(diagnostic_status.Level > NOTICE)
    {
        logger->log_error("Unable to find Serial Ports. Exiting.");
        printf("[MasterNode]: Unable to find Serial Ports. Exiting.\n");
        return false;
    }

    if(check_serialports() == false)
    {
        logger->log_error("Unable to check Serial Ports. Exiting.");
        printf("[MasterNode]: Unable to check Serial Ports. Exiting.\n");
        return false;
    }
    else
    {
    	logger->log_notice("Serial Port Check Complete.");
    	//printf("[MasterNode]: Serial Port Check Complete.\n");
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
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
