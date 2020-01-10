#include "master_node.h"
bool kill_node = false;
bool MasterNode::start(int argc,char **argv)
{

	bool status = false;
	last_armedstate = ARMEDSTATUS_UNDEFINED;
	process = new MasterNodeProcess();
	disable_readytoarm_publisher();
	serialmessagehandler = new SerialMessageHandler;
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
	diagnostic = preinitialize_basenode(argc,argv);
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if(diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(),get_nodename(),get_hostname(),DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	write_logfile("Master Node Starting.");
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	process->enable_diagnostics(diagnostic_types);
	process->set_filepaths("/home/robot/config/SystemFile.xml","/home/robot/config/DeviceFile.xml");

	if(diagnostic.Level > WARN)
	{
		return false;
	}
	system("rosnode list -ua > /home/robot/unsorted && sort /home/robot/unsorted > /home/robot/config/AllNodeList");

	bool create_nodelist = process->create_nodelist("/home/robot/config/AllNodeList","/home/robot/config/ActiveNodes");
	if(create_nodelist == false)
	{
		logger->log_error(__FILE__,__LINE__,"Couldn't initialize ActiveNodeList. Exiting.\n");
		return false;
	}

	diagnostic = process->set_serialportlist(find_serialports());
	if(diagnostic.Level > NOTICE)
	{
		logger->log_error(__FILE__,__LINE__,"Unable to find Serial Ports. Exiting.");
		return false;
	}

	if(check_serialports() == false)
	{
		logger->log_error(__FILE__,__LINE__,"Unable to check Serial Ports. Exiting.");
		return false;
	}
	else
	{
		logger->log_notice(__FILE__,__LINE__,"Serial Port Check Complete.");
	}
	diagnostic = finish_initialization();
	diagnostic = process->finish_initialization();
	set_mydevice(process->get_mydevice());
	print_deviceinfo();
	if(diagnostic.Level < WARN)
	{
		diagnostic = process->update_diagnostic(diagnostic);
		get_logger()->log_diagnostic(diagnostic);
	}

	status = true;
	return status;
}

eros::diagnostic MasterNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice(__FILE__,__LINE__,"Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic MasterNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&MasterNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&MasterNode::Command_Callback,this);
	std::string srv_device_topic = "/" + node_name + "/srv_device";
	device_srv = n->advertiseService(srv_device_topic,&MasterNode::device_service,this);
	std::string srv_pin_topic = "/" + node_name + "/srv_pin";
	pin_srv = n->advertiseService(srv_pin_topic,&MasterNode::pin_service,this);
	std::string srv_connection_topic = "/" + node_name + "/srv_connection";
	connection_srv = n->advertiseService(srv_connection_topic,&MasterNode::connection_service,this);
	std::string srv_leverarm_topic = "/" + node_name + "/srv_leverarm";
	leverarm_srv = n->advertiseService(srv_leverarm_topic,&MasterNode::leverarm_service,this);
	//Commenting out publish for BUG: Resource Monitor Available Resource Unstable #208
	std::string device_resourceavail_topic = "/" + process->get_mydevice().DeviceName + "/resource_available";
	device_resourceavail_pub = n->advertise<eros::resource>(device_resourceavail_topic,1);
	std::string loadfactor_topic = "/" + process->get_mydevice().DeviceName + "/loadfactor";
	loadfactor_pub = n->advertise<eros::loadfactor>(loadfactor_topic,1);
	std::string uptime_topic = "/" + process->get_mydevice().DeviceName + "/uptime";
	uptime_pub = n->advertise<eros::uptime>(uptime_topic,1);
	armedstate_sub = n->subscribe<std_msgs::UInt8>("/armed_state",1,&MasterNode::ArmedState_Callback,this);

	return diagnostic;
}
bool MasterNode::write_logfile(std::string text)
{
	ofstream logfile;
	std::string filepath = "/var/log/output/" + get_hostname() + "_log.txt";
	logfile.open (filepath, ios::out | ios::app);
	if(logfile.is_open() == false)
	{
		logger->log_error(__FILE__,__LINE__,"Could not open system log file.");
		return false;
	}
	std::string str1 = process->exec("date +%d-%b-%Y",true);
	std::string str2 = process->exec("date +%H:%M:%S",true);
	std::string str3 = process->exec("awk '{print $1}' /proc/uptime",true);
	logfile << "["  << str1.substr(0,str1.size()-1) << " " << str2.substr(0,str2.size()-1) << "/" << str3.substr(0,str3.size()-1) << "] " <<  text << std::endl;
	logfile.close();
	return true;
}
bool MasterNode::run_001hz()
{
	return true;
}
bool MasterNode::run_01hz()
{
	//Commenting out publish for BUG: Resource Monitor Available Resource Unstable #208
	eros::resource device_resource_available;
	device_resource_available.stamp = ros::Time::now();
	device_resource_available.Node_Name = process->get_mydevice().DeviceName;
	device_resource_available.PID = 0;
	device_resource_available.CPU_Perc = resourcemonitor->get_CPUFree_perc();
	device_resource_available.RAM_MB = (double)(resourcemonitor->get_RAMFree_kB()/1000.0);
	device_resource_available.RAM_Perc = resourcemonitor->get_RAMFree_perc();
	device_resource_available.DISK_Perc = resourcemonitor->get_DISKFree_perc();
	device_resourceavail_pub.publish(device_resource_available);
	process->update_diagnostic(get_resource_diagnostic());
	eros::diagnostic diag = process->slowupdate();
	if(diag.Level <= NOTICE)
	{
		loadfactor_pub.publish(process->getLoadFactor());
		uptime_pub.publish(process->getUptime());
	}
	else
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool MasterNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool MasterNode::run_1hz()
{
	if(process->get_mydevice().Architecture == "armv7l")
	{
		process->set_devicetemperature(read_device_temperature());
		
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level == WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool MasterNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level == WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool MasterNode::run_loop1()
{
	return true;
}
bool MasterNode::run_loop2()
{
	return true;
}
bool MasterNode::run_loop3()
{
	return true;
}

void MasterNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}
void MasterNode::ArmedState_Callback(const std_msgs::UInt8::ConstPtr& t_msg)
{
	if(t_msg->data != last_armedstate)
	{
		write_logfile("Armed State changed from: " + 
			process->map_armedstate_tostring(last_armedstate) + " to: " + 
			process->map_armedstate_tostring(t_msg->data));
		last_armedstate = t_msg->data;
	}
}
void MasterNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool MasterNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			
			process->set_mydevice(t_device);
		}
	}

	if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
bool MasterNode::check_serialports()
{
	std::vector<MasterNodeProcess::SerialPort> ports = process->get_serialports();
	std::vector<std::string> baudrates = process->get_allserialbaudrates();
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
				logger->log_info(__FILE__,__LINE__,std::string(tempstr));
				std::string baudrate = baudrates.at(j);
				int dev_fd = open(ports.at(i).file.c_str(),O_RDWR | O_NOCTTY);
				if(dev_fd < 0)
				{
					char tempstr[512];
					sprintf(tempstr,"Can't open: %s\n",ports.at(i).file.c_str());
					logger->log_error(__FILE__,__LINE__,std::string(tempstr));
					break;
				}
				struct termios tty;
				memset(&tty,0,sizeof tty);
				if(tcgetattr(dev_fd,&tty) != 0 )
				{
					logger->log_error(__FILE__,__LINE__,strerror(errno));
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
					char tempstr[512];
					sprintf(tempstr,"Baudrate: %s Not Supported.",baudrate.c_str());
					logger->log_error(__FILE__,__LINE__,std::string(tempstr));
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
				if ( tcsetattr ( dev_fd, TCSANOW, &tty ) != 0)
				{
					logger->log_error(__FILE__,__LINE__,strerror(errno));
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
						ros::Duration(2.0).sleep();
					}

				}
				int packets_to_check_counter = 0;
				while((packets_to_check_counter < 50) && (port_valid != true))
				{
					int n = 0;
					int length = 0;
					unsigned char response[MAX_SERIALPACKET_SIZE];
					int spot = 0;
					unsigned char buf = '\0';
					memset(response, '\0', sizeof response);
					do
					{
						n = read( dev_fd, &buf, 1 );
						length += n;
						response[spot] = buf;
						spot += n;
					} while(buf != '\n' &&  buf != '\r' && n > 0 && length < MAX_SERIALPACKET_SIZE);
					if (n < 0)
					{
						logger->log_error(__FILE__,__LINE__,strerror(errno));
					}
					else if (n == 0)
					{
						logger->log_error(__FILE__,__LINE__,"Read Nothing.");
					}
					else
					{
						if(length > 4)
						{
							bool status = process->new_serialmessage(ports.at(i).file,baudrate,response,length);
							if(status == true)
							{
								continue_checking_port = false;
								port_valid = true;
							}
							else
							{
								ros::Duration(2.0).sleep();
							}

						}
					}
					packets_to_check_counter++;
				}
				close(dev_fd);
			}
		}
	}
	ports = process->get_serialports();
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
			logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
		}
		else
		{
			char tempstr[512];
			sprintf(tempstr,"No ROS Device found on: %s\n",
					ports.at(i).file.c_str());
			logger->log_warn(__FILE__,__LINE__,std::string(tempstr));
		}
	}
	if(ports.size() == 0)
	{
		process->update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"No Serial Ports Found.");
	}
	else
	{
		process->update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"No Error.");
	}
	return true;
}
std::vector<std::string> MasterNode::find_serialports()
{
	std::vector<std::string> ports;
	std::vector<std::string> files;
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir("/dev/")) == NULL)
	{
		logger->log_error(__FILE__,__LINE__,strerror(errno));
		return ports;
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
bool MasterNode::connection_service(eros::srv_connection::Request &req,
		eros::srv_connection::Response &res)
{

	std::vector<MasterNodeProcess::SerialPort> ports = process->get_serialports();
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
bool MasterNode::pin_service(eros::srv_pin::Request &req,
		eros::srv_pin::Response &res)
{
	std::vector<eros::device> alldevices = process->get_alldevices();
	for(std::size_t i = 0; i < alldevices.size(); i++)
	{
		for(std::size_t j = 0; j < alldevices.at(i).pins.size(); ++j)
		{
			if((alldevices.at(i).pins.at(j).ConnectedDevice == req.query) ||
			   (alldevices.at(i).pins.at(j).ConnectedSensor == req.query))
			{
				res.pins.push_back(alldevices.at(i).pins.at(j));
			}
		}
	}
	return true;
}
bool MasterNode::device_service(eros::srv_device::Request &req,
		eros::srv_device::Response &res)
{
	if(req.query == "SELF")
	{
		res.data.push_back(process->get_mydevice());
		return true;
	}
	else if(std::string::npos != req.query.find("DeviceType="))
	{
		std::string devicetype = req.query.substr(11,req.query.size());
		if(std::string::npos != req.filter.find("*"))
		{
			for(std::size_t i = 0; i < process->get_alldevices().size(); i++)
			{
				if(process->get_alldevices().at(i).DeviceType == devicetype)
				{
					res.data.push_back(process->get_alldevices().at(i));
				}
			}
		}
		else
		{
			for(std::size_t i = 0; i < process->get_childdevices().size(); i++)
			{
				if(process->get_childdevices().at(i).DeviceType == devicetype)
				{
					res.data.push_back(process->get_childdevices().at(i));
				}
			}
		}
		return true;
	}
	else if(req.query == "ALL")
	{
		std::vector<eros::device> alldevices = process->get_alldevices();
		for(std::size_t i = 0; i < alldevices.size(); i++)
		{
			res.data.push_back(alldevices.at(i));
		}
		return true;

	}
	return false;
}
bool MasterNode::leverarm_service(eros::srv_leverarm::Request &req,
		eros::srv_leverarm::Response &res)
{
	eros::leverarm la;
	bool status = process->get_leverarm(&la,req.name);
	if(status == false) { return false; }
	res.lever = la;
	return true;
}
double MasterNode::read_device_temperature()
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
		logger->log_error(__FILE__,__LINE__,"Unable to read system temperature.");
	}
	return temp;
}
void MasterNode::print_deviceinfo()
{
	char tempstr[8192];
	sprintf(tempstr,"Loading Devices:\n");
	for(std::size_t i = 0; i < process->get_childdevices().size(); ++i)
	{
		sprintf(tempstr,"%s[%d] Device: %s\n",tempstr,(int)i,process->get_childdevices().at(i).DeviceName.c_str());
		eros::leverarm la;
		if(process->get_leverarm(&la,process->get_childdevices().at(i).DeviceName) == true)
		{
			sprintf(tempstr,"%s  LeverArm: Reference: %s X:%4.2f (m) Y:%4.2f (m) Z:%4.2f (m) Roll:%4.4f (deg) Pitch: %4.4f (deg) Yaw: %4.4f (deg)\n",
					tempstr,
					la.reference.c_str(),
					la.x.value,la.y.value,la.z.value,
					la.roll.value,la.pitch.value,la.yaw.value);
		}
		for(std::size_t j = 0; j < process->get_childdevices().at(i).pins.size(); ++j)
		{
			sprintf(tempstr,"%s  [%d] Pin: %s\n",tempstr,(int)j,process->get_childdevices().at(i).pins.at(j).Name.c_str());
		}
	}
	logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
}
void MasterNode::cleanup()
{

}
void MasterNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
	write_logfile("Master Node Ended.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing Node with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	MasterNode *node = new MasterNode();
	bool status = node->start(argc,argv);
	std::thread thread(&MasterNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->get_logger()->log_info(__FILE__,__LINE__,"Node Finished Safely.");
	return 0;
}

