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
	std::string param_use_calibrationfile = node_name +"/use_calibrationfile";
	bool load_calibrationfile = false;
	if(n->getParam(param_use_calibrationfile,load_calibrationfile) == false)
	{
		get_logger()->log_warn(__FILE__,__LINE__,"Param: use_calibrationfile Not Found.");
	}
	if(load_calibrationfile == false)
	{
		get_logger()->log_warn(__FILE__,__LINE__,"Not using Calibration File.");
	}
	else
	{
		get_logger()->log_info(__FILE__,__LINE__,"Using Calibration File.");
	}
	process->set_readsensorfile(load_calibrationfile);
	get_logger()->log_notice(__FILE__,__LINE__,"All Configuration Files Loaded.");
	
	return diagnostic;
}
eros::diagnostic IMUNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&IMUNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",10,&IMUNode::Command_Callback,this);
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
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
		if((process->get_imus_initialized() == true) and (process->get_imus_running() == false))
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				IMUDriver imu_driver;
				int status = imu_driver.init(imus.at(i).partnumber,imus.at(i).device_path,imus.at(i).devicename,0);
				logger->log_notice(__FILE__,__LINE__,"Init IMU: " + imus.at(i).devicename + " at Port: " + imu_driver.get_port());
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
						logger->log_notice(__FILE__,__LINE__,tempstr);
						IMUNodeProcess::IMU imu = process->get_imu(imus.at(i).devicename);
						print_imustats(imu);
					}
					else
					{
						char tempstr[512];
						sprintf(tempstr,"IMU Driver Failed to Start: %s. Exiting.",imus.at(i).devicename.c_str());
						logger->log_error(__FILE__,__LINE__,tempstr);
						kill_node = 1;
					}
				}
			}

		}
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
	}
	else if (process->get_taskstate() == TASKSTATE_INITIALIZING)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{

					get_logger()->log_error(__FILE__,__LINE__,"Got unexpected device message.");
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

			dev_srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_IMU)).c_str();
			if(srv_device.call(dev_srv) == true)
			{
				for(std::size_t i = 0; i < dev_srv.response.data.size(); i++)
				{
					eros::srv_leverarm la_srv;
					la_srv.request.name = dev_srv.response.data.at(i).DeviceName;
					char tempstr[512];
					sprintf(tempstr,"Received Device Info for: %s",dev_srv.response.data.at(i).DeviceName.c_str());
					eros::diagnostic diag = process->update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,std::string(tempstr));
					logger->log_diagnostic(diag);
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
	if(process->get_taskstate() == TASKSTATE_RUNNING)
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
	if(t_msg->Command == ROVERCOMMAND_SETLOGLEVEL)
	{
		if(t_msg->Option1 == POSE_NODE)
		{
			for(std::size_t i = 0; i < imu_drivers.size(); ++i)
			{
				imu_drivers.at(i).set_debugmode(t_msg->Option2);
			}
		}
	}
	else if(t_msg->Command == ROVERCOMMAND_CALIBRATION)
	{
		if(t_msg->Option1 == ROVERCOMMAND_CALIBRATION_MAGNETOMETER)
		{
			logger->log_notice(__FILE__,__LINE__,"Resetting Magnetometer Calibration Data.");
		}
		if(t_msg->Option1 == ROVERCOMMAND_CALIBRATION_MOUNTINGANGLEOFFSET)
		{
			logger->log_notice(__FILE__,__LINE__,"Resetting Mounting Angle Offset Data.");
		}
		std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
		for(std::size_t i = 0; i < imus.size(); ++i)
		{
			print_imustats(imus.at(i));
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

	if ((process->get_taskstate() == TASKSTATE_INITIALIZED))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
bool IMUNode::new_devicemsg(__attribute__((unused))std::string query,eros::device t_device,eros::leverarm t_leverarm)
{
	if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::leverarm::ConstPtr leverarm_ptr(new eros::leverarm(t_leverarm));
		eros::diagnostic diag = process->new_devicemsg(device_ptr,leverarm_ptr);
		if(diag.Level > NOTICE)
		{
			logger->log_diagnostic(diag);
		}
	}
	return true;
}
void IMUNode::print_imustats(IMUNodeProcess::IMU imu)
{
		{
			char tempstr[256];
			sprintf(tempstr,"IMU Stats-%s",imu.devicename.c_str());
			logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
		}
		{
			char tempstr[256];
			sprintf(tempstr,"Sensor Info File Path: %s\n",imu.sensor_info_path.c_str());
			logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
		}
		{
			print_3x3_matricies(imu.rotate_matrix);
		}
		{
			char tempstr[1024];
			sprintf(tempstr,"Magnetometer Ellipsoid Fit Rotation Matrix:\n"
					"%4.8f %4.8f %4.8f\n"
					"%4.8f %4.8f %4.8f\n"
					"%4.8f %4.8f %4.8f",
					imu.MagnetometerEllipsoidFit_RotationMatrix(0,0),
					imu.MagnetometerEllipsoidFit_RotationMatrix(0,1),
					imu.MagnetometerEllipsoidFit_RotationMatrix(0,2),
					imu.MagnetometerEllipsoidFit_RotationMatrix(1,0),
					imu.MagnetometerEllipsoidFit_RotationMatrix(1,1),
					imu.MagnetometerEllipsoidFit_RotationMatrix(1,2),
					imu.MagnetometerEllipsoidFit_RotationMatrix(2,0),
					imu.MagnetometerEllipsoidFit_RotationMatrix(2,1),
					imu.MagnetometerEllipsoidFit_RotationMatrix(2,2));
			logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
		}
		{
			char tempstr[1024];
			sprintf(tempstr,"Magnetometer Ellipsoid Fit Bias Vector:\n"
					"%4.4f %4.4f %4.4f",
					imu.MagnetometerEllipsoidFit_Bias(0),
					imu.MagnetometerEllipsoidFit_Bias(1),
					imu.MagnetometerEllipsoidFit_Bias(2));
			logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
		}
}
void IMUNode::print_3x3_matricies(IMUNodeProcess::RotationMatrix mat)
{
	{
		char tempstr[1024];
		sprintf(tempstr,"Rotation Matrix Acc:\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f",
				mat.Rotation_Acc(0,0),
				mat.Rotation_Acc(0,1),
				mat.Rotation_Acc(0,2),
				mat.Rotation_Acc(1,0),
				mat.Rotation_Acc(1,1),
				mat.Rotation_Acc(1,2),
				mat.Rotation_Acc(2,0),
				mat.Rotation_Acc(2,1),
				mat.Rotation_Acc(2,2));
		logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
	}
	{
		char tempstr[1024];
		sprintf(tempstr,"Rotation Matrix Gyro:\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f",
				mat.Rotation_Gyro(0,0),
				mat.Rotation_Gyro(0,1),
				mat.Rotation_Gyro(0,2),
				mat.Rotation_Gyro(1,0),
				mat.Rotation_Gyro(1,1),
				mat.Rotation_Gyro(1,2),
				mat.Rotation_Gyro(2,0),
				mat.Rotation_Gyro(2,1),
				mat.Rotation_Gyro(2,2));
		logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
	}
	{
		char tempstr[1024];
		sprintf(tempstr,"Rotation Matrix Mag:\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f\n"
				"%4.4f %4.4f %4.4f",
				mat.Rotation_Mag(0,0),
				mat.Rotation_Mag(0,1),
				mat.Rotation_Mag(0,2),
				mat.Rotation_Mag(1,0),
				mat.Rotation_Mag(1,1),
				mat.Rotation_Mag(1,2),
				mat.Rotation_Mag(2,0),
				mat.Rotation_Mag(2,1),
				mat.Rotation_Mag(2,2));
		logger->log_notice(__FILE__,__LINE__,std::string(tempstr));
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
	printf("Killing Node with Signal: %d", sig);
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
		status = node->update(node->get_process()->get_taskstate());
	}
	node->get_logger()->log_info(__FILE__,__LINE__,"Node Finished Safely.");
	node->cleanup();
	thread.detach();
	return 0;
}
