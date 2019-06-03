#include "imu_node.h"
bool kill_node = false;
bool IMUNode::start(int argc,char **argv)
{
	bool status = false;
	process = new IMUNodeProcess();
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
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
	{
		diagnostic = process->update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Configured.  Initializing.");
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic IMUNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic IMUNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&IMUNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&IMUNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string leverarm_topic = "/" + std::string(host_name) + "_master_node/srv_leverarm";
	srv_leverarm = n->serviceClient<eros::srv_leverarm>(leverarm_topic);
	return diagnostic;
}
bool IMUNode::run_001hz()
{
	return true;
}
bool IMUNode::run_01hz()
{
	return true;
}
bool IMUNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool IMUNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
		if((process->get_imus_initialized() == true) and (process->get_imus_running() == false))
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				IMUDriver imu_driver;
				int status = imu_driver.init(imus.at(i).partnumber,imus.at(i).device_path,imus.at(i).devicename,0);
				if(status <= 0)
				{
					diagnostic.Diagnostic_Type = SENSORS;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
					char tempstr[512];
					sprintf(tempstr,"Failed to Initialize IMU at: %s.  Exiting.",imus.at(i).device_path.c_str());
					diagnostic.Description = std::string(tempstr);
					logger->log_diagnostic(diagnostic);
					kill_node = 1;
				}
				else
				{
					{
						std::string imu_topic = "/" + imus.at(i).devicename;
						ros::Publisher pub = n->advertise<eros::imu>(imu_topic,1);
						imu_pubs.push_back(pub);
					}
					{
						std::string imu_temp_topic = "/" + imus.at(i).devicename + "_Temperature";
						ros::Publisher pub = n->advertise<eros::signal>(imu_temp_topic,1);
						imutemp_pubs.push_back(pub);
					}
					if(process->set_imu_running(imus.at(i).devicename) == true)
					{
						imu_drivers.push_back(imu_driver);
						char tempstr[512];
						sprintf(tempstr,"IMU Driver Started: %s",imus.at(i).devicename.c_str());
						logger->log_notice(tempstr);

						print_3x3_matricies(imus.at(i).devicename,process->get_imu(imus.at(i).devicename).rotate_matrix);
					}
					else
					{
						char tempstr[512];
						sprintf(tempstr,"IMU Driver Failed to Start: %s. Exiting.",imus.at(i).devicename.c_str());
						logger->log_error(tempstr);
						kill_node = 1;
					}
				}
			}

		}
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
	}
	else if(process->is_initialized() == false)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
			else
			{
			}
		}
		{
			eros::srv_device dev_srv;

			dev_srv.request.query = "DeviceType=IMU";
			if(srv_device.call(dev_srv) == true)
			{
				for(std::size_t i = 0; i < dev_srv.response.data.size(); i++)
				{
					eros::srv_leverarm la_srv;
					la_srv.request.name = dev_srv.response.data.at(i).DeviceName;
					if(srv_leverarm.call(la_srv) == true)
					{
						new_devicemsg(dev_srv.request.query,dev_srv.response.data.at(i),la_srv.response.lever);
					}

				}
			}
			else
			{
			}
		}
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
bool IMUNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1, ros::Time::now().toSec());
	if (diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level > WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	if(process->get_imureset_trigger() == true)
	{
		
		for(std::size_t i = 0; i < imu_drivers.size(); ++i)
		{
			diag = process->update_diagnostic(imu_drivers.at(i).get_devicename(),SENSORS,NOTICE,INITIALIZING,"Resetting IMU.");
			get_logger()->log_diagnostic(diag);
			bool status = imu_drivers.at(i).reset();
			if(status == false)
			{
				diag = process->update_diagnostic(imu_drivers.at(i).get_devicename(),SENSORS,ERROR,INITIALIZING_ERROR,"IMU Reset Failed.");
			}
		}

	}
	return true;
}
bool IMUNode::run_loop1()
{
	eros::diagnostic diag;
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
		if((process->get_imus_initialized() == true) and (process->get_imus_running() == true))
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			std::vector<uint64_t> serial_numbers = process->get_imu_serialnumbers();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				eros::imu proc_imu;
				eros::signal proc_imu_temp;
				diag = process->new_imumsg(imus.at(i).devicename,imu_drivers.at(i).update(),proc_imu,proc_imu_temp);
				if((serial_numbers.at(i) == 0) and (process->get_runtime() > COMM_TIMEOUT_THRESHOLD))
				{
					diag = process->update_diagnostic(imus.at(i).devicename,SENSORS,ERROR,INITIALIZING_ERROR,"Could not read Serial Number. Exiting.");
					logger->log_diagnostic(diag);
					kill_node = 1;
				}
				proc_imu.timestamp = ros::Time::now().toSec();
				imutemp_pubs.at(i).publish(proc_imu_temp);
				if(diag.Level <= NOTICE)
				{
					imu_pubs.at(i).publish(proc_imu);
				}
				else
				{
					imu_pubs.at(i).publish(proc_imu);
					diagnostic_pub.publish(diag);
				}

			}
		}
	}
	return true;
}
bool IMUNode::run_loop2()
{
	return true;
}
bool IMUNode::run_loop3()
{
	return true;
}

void IMUNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void IMUNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	if(t_msg->Command == ROVERCOMMAND_SETDEBUGLEVEL)
	{
		if(t_msg->Option1 == SENSORS)
		{
			char tempstr[128];
			sprintf(tempstr,"Setting Debug Level: %d",t_msg->Option2);
			logger->log_notice(std::string(tempstr));
			for(std::size_t i = 0; i < imu_drivers.size(); ++i)
			{
				imu_drivers.at(i).set_debugmode(t_msg->Option2);
			}
		}
	}
	new_commandmsg_result(t_msg,diaglist);
}
bool IMUNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}

	if((process->is_initialized() == true))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
bool IMUNode::new_devicemsg(std::string query,eros::device t_device,eros::leverarm t_leverarm)
{
	if((process->is_initialized() == true))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::leverarm::ConstPtr leverarm_ptr(new eros::leverarm(t_leverarm));
		eros::diagnostic diag = process->new_devicemsg(device_ptr,leverarm_ptr);
	}
	return true;
}
void IMUNode::print_3x3_matricies(std::string devicename,IMUNodeProcess::RotationMatrix mat)
{
	{
		char tempstr[1024];
		sprintf(tempstr,"%s-Rotation Matrix Acc:\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f",
				devicename.c_str(),
				mat.Rotation_Acc(0,0),
				mat.Rotation_Acc(0,1),
				mat.Rotation_Acc(0,2),
				mat.Rotation_Acc(1,0),
				mat.Rotation_Acc(1,1),
				mat.Rotation_Acc(1,2),
				mat.Rotation_Acc(2,0),
				mat.Rotation_Acc(2,1),
				mat.Rotation_Acc(2,2));
		logger->log_debug(std::string(tempstr));
	}
	{
		char tempstr[1024];
		sprintf(tempstr,"%s-Rotation Matrix Gyro:\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f",
				devicename.c_str(),
				mat.Rotation_Gyro(0,0),
				mat.Rotation_Gyro(0,1),
				mat.Rotation_Gyro(0,2),
				mat.Rotation_Gyro(1,0),
				mat.Rotation_Gyro(1,1),
				mat.Rotation_Gyro(1,2),
				mat.Rotation_Gyro(2,0),
				mat.Rotation_Gyro(2,1),
				mat.Rotation_Gyro(2,2));
		logger->log_debug(std::string(tempstr));
	}
	{
		char tempstr[1024];
		sprintf(tempstr,"%s-Rotation Matrix Mag:\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f",
				devicename.c_str(),
				mat.Rotation_Mag(0,0),
				mat.Rotation_Mag(0,1),
				mat.Rotation_Mag(0,2),
				mat.Rotation_Mag(1,0),
				mat.Rotation_Mag(1,1),
				mat.Rotation_Mag(1,2),
				mat.Rotation_Mag(2,0),
				mat.Rotation_Mag(2,1),
				mat.Rotation_Mag(2,2));
		logger->log_debug(std::string(tempstr));
	}
}
void IMUNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void IMUNode::cleanup()
{
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	IMUNode *node = new IMUNode();
	bool status = node->start(argc,argv);
	std::thread thread(&IMUNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

