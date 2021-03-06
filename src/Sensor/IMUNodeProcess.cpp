#include "IMUNodeProcess.h"
eros::diagnostic  IMUNodeProcess::finish_initialization()
{
	supported_partnumbers.push_back(PN_110012);
	supported_partnumbers.push_back(PN_110015);
	reset();
	eros::diagnostic diag = root_diagnostic;
	imus_initialized = false;
	imus_running = false;
	return diag;
}
eros::diagnostic IMUNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	//Task State Machine Updates
	if(task_state == TASKSTATE_PAUSE)
	{

	}
	else if(task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
		
	}
	else if(task_state == TASKSTATE_INITIALIZED)
	{
		if(mydevice.SensorCount == 0)
		{
			request_statechange(TASKSTATE_RUNNING);
		}
	}
	else if(task_state == TASKSTATE_RUNNING)
	{
	}
	else if(task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if(v == false)
		{
			diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
				"Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	diag = update_baseprocess(t_dt,t_ros_time);
	if(task_state == TASKSTATE_RUNNING)
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");

	}
	
	diag = update_baseprocess(t_dt,t_ros_time);
	if(diag.Level > NOTICE)
	{
		return diag;
	}
	bool ok = true;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).running == false)
		{
			ok = false;
		}
		else if((imus.at(i).running == true) and (run_time > IMU_INVALID_TIME_THRESHOLD))
		{
			double dt = run_time - imus.at(i).lasttime_rx;
			
			if(imus.at(i).update_count == 0)
			{
				eros::diagnostic imu_diagnostic = imus.at(i).diagnostic;
				imu_diagnostic.Diagnostic_Type = SENSORS;
				imu_diagnostic.Level = ERROR;
				imu_diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
				char tempstr[512];
				sprintf(tempstr,"Never Received data from IMU: %s",imus.at(i).devicename.c_str());
				imu_diagnostic.Description = std::string(tempstr);
				imus.at(i).diagnostic = imu_diagnostic;
				diag = update_diagnostic(imus.at(i).devicename,SENSORS,ERROR,DEVICE_NOT_AVAILABLE,std::string(tempstr));
				ok = false;
			}
			else if(dt >=IMU_INVALID_TIME_THRESHOLD)
			{
				if(imu_reset_inprogress == false)
				{
					imu_reset_trigger = true;
					imu_reset_inprogress = true;
				}
				if(imu_reset_inprogress == true)
				{
					if(imu_reset_inprogress_timer > 2.0*IMU_INVALID_TIME_THRESHOLD)
					{
						imu_reset_trigger = true;
					}
				}
				eros::diagnostic imu_diagnostic = imus.at(i).diagnostic;
				imu_diagnostic.Diagnostic_Type = SENSORS;
				imu_diagnostic.Level = WARN;
				imu_diagnostic.Diagnostic_Message = DROPPING_PACKETS;
				char tempstr[512];
				sprintf(tempstr,"No IMU Data from %s in %f Seconds. Update Rate: %4.2fHz",imus.at(i).devicename.c_str(),dt,imus.at(i).update_rate);
				imu_diagnostic.Description = std::string(tempstr);
				imus.at(i).diagnostic = imu_diagnostic;
				diag = update_diagnostic(imus.at(i).devicename,imu_diagnostic.Diagnostic_Type,imu_diagnostic.Level,imu_diagnostic.Diagnostic_Message,std::string(tempstr));
				ok = false;
			}
			else
			{
				eros::diagnostic imu_diagnostic = imus.at(i).diagnostic;
				imu_diagnostic.Diagnostic_Type = SENSORS;
				imu_diagnostic.Level = INFO;
				imu_diagnostic.Diagnostic_Message = NOERROR;
				char tempstr[512];
				sprintf(tempstr,"IMU: %s Updated, Update Rate: %4.2fHz",imus.at(i).devicename.c_str(),imus.at(i).update_rate);
				imu_diagnostic.Description = std::string(tempstr);
				imus.at(i).diagnostic = imu_diagnostic;
				diag = update_diagnostic(imus.at(i).devicename,SENSORS,INFO,NOERROR,std::string(tempstr));
			}
		}
		else
		{
			ok = ok and true;
		}
	}
	if(imus.size() == 0)
	{
		ok = false;
		diag = update_diagnostic(SENSORS,WARN,DEVICE_NOT_AVAILABLE,"No IMU's Configured.");
	}
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");

	}
	if(ok == true)
	{
		ready_to_arm = true;
	}
	else
	{
		ready_to_arm = false;
	}
	return diag;
}
eros::diagnostic IMUNodeProcess::new_imumsg(std::string devicename,IMUDriver::RawIMU raw_data,eros::imu &imu_output,eros::signal &proc_imu_temperature)
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{

			if((raw_data.signal_state == SIGNALSTATE_UPDATED) or 
			   (raw_data.signal_state == SIGNALSTATE_EXTRAPOLATED) or
			   (raw_data.signal_state == SIGNALSTATE_HOLD) 
			  )
			  {
				  imu_reset_inprogress = false;
				  imu_reset_inprogress_timer = 0.0;
			  }
			else
			{
				char tempstr[512];
				sprintf(tempstr,"IMU State: %s",map_signalstate_tostring(raw_data.signal_state).c_str());
				diag = update_diagnostic(devicename,SENSORS,WARN,DROPPING_PACKETS,std::string(tempstr));
				return diag;
			}
			if(raw_data.serial_number != 0)
			{
				if(imus.at(i).serial_number_checked == false)
				{
				if(raw_data.serial_number != imus.at(i).serial_number)
				{
					char tempstr[512];
					sprintf(tempstr,"Expected Id: %llu but Received: %llu",(long long unsigned)imus.at(i).serial_number,(long long unsigned)raw_data.serial_number);
					diag = update_diagnostic(imus.at(i).devicename,DATA_STORAGE,ERROR,DEVICE_NOT_AVAILABLE,std::string(tempstr));
				}
				else
				{
					diag = update_diagnostic(imus.at(i).devicename,DATA_STORAGE,INFO,NOERROR,"Serial Number Checked.");
				}
				}
				imus.at(i).serial_number_checked = true;
			}
			
			imu_output = imus.at(i).imu_data;
			imus.at(i).packet_count++;
			imus.at(i).sequence_number = raw_data.sequence_number;
			imus.at(i).imu_data.tov = raw_data.tov;
			imu_output.tov = raw_data.tov;
			proc_imu_temperature = convert_signal(raw_data.temperature);
			imu_output.xacc = convert_signal(raw_data.acc_x);
			imu_output.yacc = convert_signal(raw_data.acc_y);
			imu_output.zacc = convert_signal(raw_data.acc_z);
			imu_output.xgyro = convert_signal(raw_data.gyro_x);
			imu_output.ygyro = convert_signal(raw_data.gyro_y);
			imu_output.zgyro = convert_signal(raw_data.gyro_z);
			imu_output.xmag = convert_signal(raw_data.mag_x);
			imu_output.ymag = convert_signal(raw_data.mag_y);
			imu_output.zmag = convert_signal(raw_data.mag_z);
			Eigen::Vector3f mag(3);
			mag << imu_output.xmag.value,imu_output.ymag.value,imu_output.zmag.value;
			for(int j = 0; j < 3; ++j)
			{
				//printf("i: %d xxx1: %4.4f\n",j,mag(j));
				mag(j) = mag(j)-imus.at(i).MagnetometerEllipsoidFit_Offset(j);
				//printf("i: %d xxx2: %4.4f\n",j,mag(j));
				mag(j) = mag(j)*imus.at(i).MagnetometerEllipsoidFit_Scale(j);
				//printf("i: %d xxx3: %4.4f\n",j,mag(j));
			}
			//printf("d: %4.4f %4.4f %4.4f\n",mag(0),mag(1),mag(2));
			mag = imus.at(i).MagnetometerEllipsoidFit_RotationMatrix*(mag-imus.at(i).MagnetometerEllipsoidFit_Bias);
			//printf("In: %4.4f %4.4f %4.4f Out: %4.4f %4.4f %4.4f\n",
			//	imu_output.xmag.value,imu_output.ymag.value,imu_output.zmag.value,
			//	mag(0),mag(1),mag(2));
			imu_output.xmag.value = mag(0);
			imu_output.ymag.value = mag(1);
			imu_output.zmag.value = mag(2);
			imu_output.sequence_number = raw_data.sequence_number;
			{
				double x = raw_data.acc_x.value/imus.at(i).acc_scale_factor;
				double y = raw_data.acc_y.value/imus.at(i).acc_scale_factor;
				double z = raw_data.acc_z.value/imus.at(i).acc_scale_factor;
				Eigen::RowVector3f v;
				v << x,y,z;
				Eigen::RowVector3f vp = imus.at(i).rotate_matrix.Rotation_Acc*v.transpose();
				imu_output.xacc.value = vp(0);
				imu_output.yacc.value = vp(1);
				imu_output.zacc.value = vp(2);
				imus.at(i).xacc_rms = compute_rms(imus.at(i).xacc_rms,imu_output.xacc.value,imus.at(i).update_count);
				imus.at(i).yacc_rms = compute_rms(imus.at(i).yacc_rms,imu_output.yacc.value,imus.at(i).update_count);
				imus.at(i).zacc_rms = compute_rms(imus.at(i).zacc_rms,imu_output.zacc.value,imus.at(i).update_count);
				imu_output.xacc.rms = imus.at(i).xacc_rms.value;
				imu_output.yacc.rms = imus.at(i).yacc_rms.value;
				imu_output.zacc.rms = imus.at(i).zacc_rms.value;
			}

			{
				double x = raw_data.gyro_x.value/imus.at(i).gyro_scale_factor;
				double y = raw_data.gyro_y.value/imus.at(i).gyro_scale_factor;
				double z = raw_data.gyro_z.value/imus.at(i).gyro_scale_factor;
				Eigen::RowVector3f v;
				v << x,y,z;
				Eigen::RowVector3f vp = imus.at(i).rotate_matrix.Rotation_Gyro*v.transpose();
				imu_output.xgyro.value = vp(0);
				imu_output.ygyro.value = vp(1);
				imu_output.zgyro.value = vp(2);
				imus.at(i).xgyro_rms = compute_rms(imus.at(i).xgyro_rms,imu_output.xgyro.value,imus.at(i).update_count);
				imus.at(i).ygyro_rms = compute_rms(imus.at(i).ygyro_rms,imu_output.ygyro.value,imus.at(i).update_count);
				imus.at(i).zgyro_rms = compute_rms(imus.at(i).zgyro_rms,imu_output.zgyro.value,imus.at(i).update_count);
				imu_output.xgyro.rms = imus.at(i).xgyro_rms.value;
				imu_output.ygyro.rms = imus.at(i).ygyro_rms.value;
				imu_output.zgyro.rms = imus.at(i).zgyro_rms.value;
			}

			{
				double x = imu_output.xmag.value/imus.at(i).mag_scale_factor;
				double y = imu_output.ymag.value/imus.at(i).mag_scale_factor;
				double z = imu_output.zmag.value/imus.at(i).mag_scale_factor;
				Eigen::RowVector3f v;
				v << x,y,z;
				Eigen::RowVector3f vp = imus.at(i).rotate_matrix.Rotation_Mag*v.transpose();
				double abs_v = sqrt((vp(0)*vp(0))+(vp(1)*vp(1))+(vp(2)*vp(2)));
				if((abs_v > MAGNETOMETER_MAGNITUDE_UPPERBOUND) or 
				   (abs_v < MAGNETOMETER_MAGNITUDE_LOWERBOUND))
				{
					char tempstr[1024];
					sprintf(tempstr,"Magnetometer absolute value: %4.4f, likely requires calibration. Input: X:%4.4f Y:%4.4f Z:%4.4f Output: X:%4.4f Y: %4.4f Z: %4.4f",
					abs_v,raw_data.mag_x.value,raw_data.mag_y.value,raw_data.mag_z.value,x,y,z);
					diag = update_diagnostic(imus.at(i).devicename,SENSORS,WARN,DIAGNOSTIC_FAILED,std::string(tempstr));
				}
				else
				{
					diag = update_diagnostic(imus.at(i).devicename,SENSORS,INFO,DIAGNOSTIC_PASSED,"No Error");
				}
				imu_output.xmag.value = vp(0);
				imu_output.ymag.value = vp(1);
				imu_output.zmag.value = vp(2);
				imus.at(i).xmag_rms = compute_rms(imus.at(i).xmag_rms,imu_output.xmag.value,imus.at(i).update_count);
				imus.at(i).ymag_rms = compute_rms(imus.at(i).ymag_rms,imu_output.ymag.value,imus.at(i).update_count);
				imus.at(i).zmag_rms = compute_rms(imus.at(i).zmag_rms,imu_output.zmag.value,imus.at(i).update_count);
				imu_output.xmag.rms = imus.at(i).xmag_rms.value;
				imu_output.ymag.rms = imus.at(i).ymag_rms.value;
				imu_output.zmag.rms = imus.at(i).zmag_rms.value;	
			}
			imus.at(i).lasttime_rx = run_time;
			imus.at(i).update_count++;
			imus.at(i).update_rate = (double)(imus.at(i).update_count)/run_time;
			imus.at(i).imu_data = imu_output;
			found = true;
		}
	}
	if(found == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Could not find Device: %s",devicename.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,DEVICE_NOT_AVAILABLE,std::string(tempstr));
	}
	return diag;
}
IMUNodeProcess::RMS IMUNodeProcess::compute_rms(RMS rms,double value,uint64_t update_count)
{
	if(update_count == 0)
	{
		rms.mean = pow(value,2.0);
	}
	else
	{
		double a = (pow(value,2.0)/(double)((update_count+1)));
		double b = (rms.mean/(double)(update_count+1));
		rms.mean += a-b;
	}
	rms.value = pow(rms.mean,0.5);
	return rms;
}
eros::diagnostic IMUNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	char tempstr[512];
	sprintf(tempstr,"DeviceType: %s Not Supported.",device->DeviceType.c_str());
	diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	return diag;
}
eros::diagnostic IMUNodeProcess::new_devicemsg(const eros::device::ConstPtr& device,const eros::leverarm::ConstPtr& leverarm,bool override_config=false,std::string override_config_path="")
{
	eros::diagnostic diag = root_diagnostic;
	if(device->DeviceType == DEVICETYPE_IMU)
	{
		if(device->PartNumber == PN_110012)
		{
			IMUNodeProcess::IMU newimu;
			newimu.initialized = false;
			newimu.devicename = device->DeviceName;
			newimu.connection_method = "serial";
			newimu.device_path = "/dev/ttyAMA0";
			newimu.comm_rate = "115200";
			newimu.partnumber = PN_110012;
			newimu.acc_scale_factor = 2000.0;
			newimu.gyro_scale_factor = 2000.0;
			newimu.mag_scale_factor = 6.665;

			newimu.update_count = 0;
			newimu.packet_count = 0;
			newimu.update_rate = 0.0;
			newimu.lasttime_rx = 0.0;
			newimu.diagnostic.DeviceName = device->DeviceName;
			newimu.serial_number = (uint64_t)device->ID;
			newimu.diagnostic.Node_Name = diag.Node_Name;
			newimu.diagnostic.System = diag.System;
			newimu.diagnostic.SubSystem = diag.SubSystem;
			newimu.diagnostic.Component = diag.Component;
			newimu.diagnostic.Diagnostic_Type = SENSORS;
			newimu.diagnostic.Level = NOTICE;
			newimu.diagnostic.Diagnostic_Message = INITIALIZING;
			newimu.diagnostic.Description = "IMU Initialized.";
			newimu.mounting_angle_offset_pitch_deg = leverarm->pitch.value;
			newimu.mounting_angle_offset_roll_deg = leverarm->roll.value;
			newimu.mounting_angle_offset_yaw_deg = leverarm->yaw.value;
			newimu.MagnetometerEllipsoidFit_RotationMatrix = Eigen::Matrix3f::Identity(3,3);
			newimu.MagnetometerEllipsoidFit_Bias = Eigen::Vector3f::Zero(3);
			newimu.MagnetometerEllipsoidFit_Offset = Eigen::Vector3f::Zero(3);
			newimu.MagnetometerEllipsoidFit_Scale = Eigen::Vector3f::Ones(3);
			newimu.rotate_matrix = generate_rotation_matrix(newimu.mounting_angle_offset_roll_deg,newimu.mounting_angle_offset_pitch_deg,newimu.mounting_angle_offset_yaw_deg);
			newimu.initialized = true;

			if(override_config == true)
			{
				newimu.sensor_info_path = override_config_path;
			}
			else
			{
				newimu.sensor_info_path = "/home/robot/config/sensors/" + device->PartNumber + "-" + std::to_string(device->ID) + "/" + device->PartNumber + "-" + std::to_string(device->ID) + ".xml";
			}
			imus.push_back(newimu);
			if(load_sensorfile == true)
			{
				diag = load_sensorinfo(device->DeviceName);
			}
			else
			{
				diag = update_diagnostic(DATA_STORAGE,NOTICE,NOERROR,"Not Loading Sensor File.");
			}
			diag = update_diagnostic(diag);
			if(diag.Level > WARN) { return diag; }
			request_statechange(TASKSTATE_RUNNING);
			imus_initialized = true;
			diag = update_diagnostic(SENSORS,INFO,NOERROR,"Sensors Initialized.");
			diag = update_diagnostic(newimu.diagnostic.DeviceName,DATA_STORAGE,INFO,INITIALIZING,"Initializing.");
			diag = update_diagnostic(newimu.diagnostic);
		}
		else if(device->PartNumber == PN_110015)
		{
			IMUNodeProcess::IMU newimu;
			newimu.initialized = false;
			newimu.devicename = device->DeviceName;
			newimu.serial_number = (uint64_t)device->ID;
			newimu.serial_number_checked = false;
			newimu.connection_method = "serial";
			newimu.device_path = "/dev/ttyACM0";
			newimu.comm_rate = "115200";
			newimu.partnumber = PN_110015;
			newimu.acc_scale_factor = 1.0;
			newimu.gyro_scale_factor = 1.0;
			newimu.mag_scale_factor = 1.0;

			newimu.update_count = 0;
			newimu.packet_count = 0;
			newimu.update_rate = 0.0;
			newimu.lasttime_rx = 0.0;

			newimu.diagnostic.DeviceName = device->DeviceName;
			newimu.diagnostic.Node_Name = diag.Node_Name;
			newimu.diagnostic.System = diag.System;
			newimu.diagnostic.SubSystem = diag.SubSystem;
			newimu.diagnostic.Component = diag.Component;
			newimu.diagnostic.Diagnostic_Type = SENSORS;
			newimu.diagnostic.Level = NOTICE;
			newimu.diagnostic.Diagnostic_Message = INITIALIZING;
			newimu.diagnostic.Description = "IMU Initialized.";
			newimu.mounting_angle_offset_pitch_deg = leverarm->pitch.value;
			newimu.mounting_angle_offset_roll_deg = leverarm->roll.value;
			newimu.mounting_angle_offset_yaw_deg = leverarm->yaw.value;
			newimu.MagnetometerEllipsoidFit_RotationMatrix = Eigen::Matrix3f::Identity(3,3);
			newimu.MagnetometerEllipsoidFit_Bias = Eigen::Vector3f::Zero(3);
			newimu.MagnetometerEllipsoidFit_Offset = Eigen::Vector3f::Zero(3);
			newimu.MagnetometerEllipsoidFit_Scale = Eigen::Vector3f::Ones(3);
			newimu.rotate_matrix = generate_rotation_matrix(newimu.mounting_angle_offset_roll_deg,newimu.mounting_angle_offset_pitch_deg,newimu.mounting_angle_offset_yaw_deg);
			newimu.initialized = true;

			if(override_config == true)
			{
				newimu.sensor_info_path = override_config_path;
			}
			else
			{
				newimu.sensor_info_path = "/home/robot/config/sensors/" + device->PartNumber + "-" + std::to_string(device->ID) + "/" + device->PartNumber + "-" + std::to_string(device->ID) + ".xml";
			}
			imus.push_back(newimu);
			if(load_sensorfile == true)
			{
				diag = load_sensorinfo(device->DeviceName);
			}
			else
			{
				diag = update_diagnostic(DATA_STORAGE,NOTICE,NOERROR,"Not Loading Sensor File.");
			}
			if(diag.Level > WARN) { return diag; }
			request_statechange(TASKSTATE_RUNNING);
			imus_initialized = true;
			diag = update_diagnostic(SENSORS,INFO,NOERROR,"Sensors Initialized.");
			diag = update_diagnostic(newimu.diagnostic.DeviceName,DATA_STORAGE,INFO,INITIALIZING,"Initializing.");
			diag = update_diagnostic(newimu.diagnostic);
		}
		else
		{
			char tempstr[512];
			sprintf(tempstr,"PartNumber: %s Not Supported.",device->PartNumber.c_str());
			diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
		}

	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"DeviceType: %s Not Supported.",device->DeviceType.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	}
	return diag;
}
bool IMUNodeProcess::set_imu_running(std::string devicename)
{
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			imus.at(i).running = true;
			imus_running = true;
			return true;
		}
	}
	return false;
}
bool IMUNodeProcess::set_imu_mounting_angles(std::string devicename,double roll_deg,double pitch_deg,double yaw_deg)
{
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			imus.at(i).mounting_angle_offset_pitch_deg = pitch_deg;
			imus.at(i).mounting_angle_offset_roll_deg = roll_deg;
			imus.at(i).mounting_angle_offset_yaw_deg = yaw_deg;
			imus.at(i).rotate_matrix = generate_rotation_matrix(roll_deg,pitch_deg,yaw_deg);
			return true;
		}
	}
	return false;
}
IMUNodeProcess::IMU IMUNodeProcess::get_imu(std::string devicename)
{
	IMUNodeProcess::IMU imu;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			return imus.at(i);
		}
	}
	imu.devicename = "";
	return imu;
}
std::vector<eros::diagnostic> IMUNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	else if(t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if(node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if(v == false)
			{
				diag = update_diagnostic(SOFTWARE,ERROR,DIAGNOSTIC_FAILED,
					"Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if(task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE,NOTICE,DIAGNOSTIC_PASSED,
					"Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}

		}
	}
	else if(t_msg->Command == ROVERCOMMAND_RESET)
	{
		if(t_msg->Option1 == SENSORS)
		{
			if(imu_reset_inprogress == false)
			{
				imu_reset_trigger = true;
				imu_reset_inprogress = true;
			}
		}
	}
	else if(t_msg->Command == ROVERCOMMAND_CALIBRATION)
	{
		if(t_msg->Option1 == ROVERCOMMAND_CALIBRATION_MAGNETOMETER)
		{
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				imus.at(i).MagnetometerEllipsoidFit_RotationMatrix = Eigen::Matrix3f::Identity(3,3);
				imus.at(i).MagnetometerEllipsoidFit_Bias = Eigen::Vector3f::Zero(3);
				imus.at(i).MagnetometerEllipsoidFit_Offset = Eigen::Vector3f::Zero(3);
				imus.at(i).MagnetometerEllipsoidFit_Scale = Eigen::Vector3f::Ones(3);
			}
		}
		else if(t_msg->Option1 == ROVERCOMMAND_CALIBRATION_MOUNTINGANGLEOFFSET)
		{
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				imus.at(i).mounting_angle_offset_roll_deg = 0.0;
				imus.at(i).mounting_angle_offset_pitch_deg = 0.0;
				imus.at(i).mounting_angle_offset_yaw_deg = 0.0;
				imus.at(i).rotate_matrix = generate_rotation_matrix(
						imus.at(i).mounting_angle_offset_roll_deg,
						imus.at(i).mounting_angle_offset_pitch_deg,
						imus.at(i).mounting_angle_offset_yaw_deg);
			}
		}
	}
	return diaglist;
}
std::vector<eros::diagnostic> IMUNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED,"Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
IMUNodeProcess::RotationMatrix IMUNodeProcess::generate_rotation_matrix(double mao_roll_deg,double mao_pitch_deg,double mao_yaw_deg)
{
	double roll = mao_roll_deg*M_PI/180.0;
	double pitch = mao_pitch_deg*M_PI/180.0;
	double yaw = mao_yaw_deg*M_PI/180.0;

	RotationMatrix R;

	R.Rotation_Acc_X.row(0) << 1.0,0.0,0.0;
	R.Rotation_Acc_X.row(1) << 0.0,cos(roll),sin(roll);
	R.Rotation_Acc_X.row(2) << 0.0,-sin(roll),cos(roll);

	R.Rotation_Acc_Y.row(0) << cos(pitch),0.0,sin(pitch);
	R.Rotation_Acc_Y.row(1) << 0.0,1.0,0.0;
	R.Rotation_Acc_Y.row(2) << -sin(pitch),0.0,cos(pitch);

	R.Rotation_Acc_Z.row(0) << cos(yaw),sin(yaw),0.0;
	R.Rotation_Acc_Z.row(1) << -sin(yaw),cos(yaw),0.0;
	R.Rotation_Acc_Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Gyro_X.row(0) << 1.0,0.0,0.0;
	R.Rotation_Gyro_X.row(1) << 0.0,cos(roll),sin(roll);
	R.Rotation_Gyro_X.row(2) << 0.0,-sin(roll),cos(roll);

	R.Rotation_Gyro_Y.row(0) << cos(pitch),0.0,sin(pitch);
	R.Rotation_Gyro_Y.row(1) << 0.0,1.0,0.0;
	R.Rotation_Gyro_Y.row(2) << -sin(pitch),0.0,cos(pitch);

	R.Rotation_Gyro_Z.row(0) << cos(yaw),sin(yaw),0.0;
	R.Rotation_Gyro_Z.row(1) << -sin(yaw),cos(yaw),0.0;
	R.Rotation_Gyro_Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Mag_X.row(0) << 1.0,0.0,0.0;
	R.Rotation_Mag_X.row(1) << 0.0,cos(roll),sin(roll);
	R.Rotation_Mag_X.row(2) << 0.0,-sin(roll),cos(roll);

	R.Rotation_Mag_Y.row(0) << cos(pitch),0.0,sin(pitch);
	R.Rotation_Mag_Y.row(1) << 0.0,1.0,0.0;
	R.Rotation_Mag_Y.row(2) << -sin(pitch),0.0,cos(pitch);

	R.Rotation_Mag_Z.row(0) << cos(yaw),sin(yaw),0.0;
	R.Rotation_Mag_Z.row(1) << -sin(yaw),cos(yaw),0.0;
	R.Rotation_Mag_Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Acc = R.Rotation_Acc_Z*R.Rotation_Acc_Y*R.Rotation_Acc_X;
	R.Rotation_Gyro = R.Rotation_Gyro_Z*R.Rotation_Gyro_Y*R.Rotation_Gyro_X;
	R.Rotation_Mag = R.Rotation_Mag_Z*R.Rotation_Mag_Y*R.Rotation_Mag_X;
	return R;
}
/*! \brief Load Sensor Info from Config
 */
eros::diagnostic IMUNodeProcess::load_sensorinfo(std::string devicename)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			uint16_t load_count = 0;
			uint16_t items_to_load = 4;
			TiXmlDocument doc(imus.at(i).sensor_info_path);
			bool loaded = doc.LoadFile();
			if(loaded == false)
			{
				char tempstr[512];
				sprintf(tempstr,"[%s]: Unable to load Sensor Config at: %s",devicename.c_str(),imus.at(i).sensor_info_path.c_str());
				diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
				return diag;
			}
			TiXmlElement *l_pRootElement = doc.RootElement();
			if(NULL != l_pRootElement)
			{
				TiXmlElement *l_pStatistics = l_pRootElement->FirstChildElement("Statistics");

				if (NULL != l_pStatistics)
				{
					//General Tag
					TiXmlElement *l_pGeneral = l_pStatistics->FirstChildElement("General");
					if (NULL != l_pGeneral)
					{

					}

					//Accelerometer Tag
					TiXmlElement *l_pAccelerometer = l_pStatistics->FirstChildElement("Accelerometer");
					if (NULL != l_pAccelerometer)
					{

					}
					//Gyroscope Tag
					TiXmlElement *l_pGyroscope = l_pStatistics->FirstChildElement("Gyroscope");
					if (NULL != l_pGyroscope)
					{

					}
					//Magnetometer Tag
					TiXmlElement *l_pMagnetometer = l_pStatistics->FirstChildElement("Magnetometer");
					if (NULL != l_pMagnetometer)
					{
						//EllipsoidFit Tag
						TiXmlElement *l_pEllipsoidFit = l_pMagnetometer->FirstChildElement("EllipsoidFit");
						if (NULL != l_pEllipsoidFit)
						{
							TiXmlElement *l_pRotationMatrix = l_pEllipsoidFit->FirstChildElement("RotationMatrix");
							if(NULL != l_pRotationMatrix)
							{
								std::string mat_string = l_pRotationMatrix->GetText();
								std::vector<std::string> elements;
								boost::split(elements, mat_string, boost::is_any_of(" "));
								if(elements.size() != 9)
								{
									char tempstr[512];
									sprintf(tempstr,"[%s]: Not all data found in Magnetometer->EllipsoidFit->RotationMatrix Tag",devicename.c_str());
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
									return diag;
								}
								int j = 0;
								int k = 0;
								Eigen::Matrix3f rotate;
								for(std::size_t i = 0; i < elements.size(); ++i)
								{
									rotate(k,j) = std::atof(elements.at(i).c_str());
									j++;
									if(j == 3)
									{
										k++;
										j=0;
									}
								}
								imus.at(i).MagnetometerEllipsoidFit_RotationMatrix = rotate;
								load_count++;
							}
							TiXmlElement *l_pBias = l_pEllipsoidFit->FirstChildElement("Bias");
							if(NULL != l_pBias)
							{
								std::string mat_string = l_pBias->GetText();
								std::vector<std::string> elements;
								boost::split(elements, mat_string, boost::is_any_of(" "));
								if(elements.size() != 3)
								{
									char tempstr[512];
									sprintf(tempstr,"[%s]: Not all data found in Magnetometer->EllipsoidFit->Bias Tag",devicename.c_str());
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
									return diag;
								}
								Eigen::Vector3f bias;
								for(std::size_t i = 0; i < elements.size(); ++i)
								{
									bias((int)i) = std::atof(elements.at(i).c_str());
								}
								imus.at(i).MagnetometerEllipsoidFit_Bias = bias;
								load_count++;
							}
							TiXmlElement *l_pScale = l_pEllipsoidFit->FirstChildElement("Scale");
							if(NULL != l_pScale)
							{
								std::string mat_string = l_pScale->GetText();
								std::vector<std::string> elements;
								boost::split(elements, mat_string, boost::is_any_of(" "));
								if(elements.size() != 3)
								{
									char tempstr[512];
									sprintf(tempstr,"[%s]: Not all data found in Magnetometer->EllipsoidFit->Scale Tag",devicename.c_str());
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
									return diag;
								}
								Eigen::Vector3f scale;
								for(std::size_t i = 0; i < elements.size(); ++i)
								{
									scale((int)i) = std::atof(elements.at(i).c_str());
								}
								imus.at(i).MagnetometerEllipsoidFit_Scale = scale;
								load_count++;
							}
							TiXmlElement *l_pOffset = l_pEllipsoidFit->FirstChildElement("Offset");
							if(NULL != l_pOffset)
							{
								std::string mat_string = l_pOffset->GetText();
								std::vector<std::string> elements;
								boost::split(elements, mat_string, boost::is_any_of(" "));
								if(elements.size() != 3)
								{
									char tempstr[512];
									sprintf(tempstr,"[%s]: Not all data found in Magnetometer->EllipsoidFit->Offset Tag",devicename.c_str());
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
									return diag;
								}
								Eigen::Vector3f offset;
								for(std::size_t i = 0; i < elements.size(); ++i)
								{
									offset((int)i) = std::atof(elements.at(i).c_str());
								}
								imus.at(i).MagnetometerEllipsoidFit_Offset = offset;
								load_count++;
							}
						}
					}
				}

				if(load_count != items_to_load)
				{
					char tempstr[512];
					sprintf(tempstr,"[%s]: Not all data present in: %s",devicename.c_str(),imus.at(i).sensor_info_path.c_str());
					diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
					return diag;
				}

				/*
				TiXmlElement *l_proll_mao = l_pRootElement->FirstChildElement("MountingAngleRoll");
				if(NULL != l_proll_mao)
				{
					mao_roll = std::atof(l_proll_mao->GetText());
				}
				else { return false; }

				TiXmlElement *l_ppitch_mao = l_pRootElement->FirstChildElement("MountingAnglePitch");
				if(NULL != l_ppitch_mao)
				{
					mao_pitch = std::atof(l_ppitch_mao->GetText());
				}
				else { return false; }

				TiXmlElement *l_pyaw_mao = l_pRootElement->FirstChildElement("MountingAngleYaw");
				if(NULL != l_proll_mao)
				{
					mao_yaw = std::atof(l_pyaw_mao->GetText());
				}
				else { return false; }
				 */

			}
			else
			{
				char tempstr[512];
				sprintf(tempstr,"[%s]: Unable to parse Sensor Config at: %s",devicename.c_str(),imus.at(i).sensor_info_path.c_str());
				diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
				return diag;
			}
			char tempstr[512];
			sprintf(tempstr,"[%s]: Loaded and Parsed Sensor Config at: %s",devicename.c_str(),imus.at(i).sensor_info_path.c_str());
			diag = update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,std::string(tempstr));
			return diag;
		}
	}
	char tempstr[512];
	sprintf(tempstr,"[%s]: Unable to find Sensor Config.",devicename.c_str());
	diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	return diag;
}
std::string IMUNodeProcess::map_signalstate_tostring(uint8_t v)
{
	switch(v)
	{
	case SIGNALSTATE_UNDEFINED:
		return "UNDEFINED";
		break;
	case SIGNALSTATE_INVALID:
		return "INVALID";
		break;
	case SIGNALSTATE_INITIALIZING:
		return "INITIALIZING";
		break;
	case SIGNALSTATE_UPDATED:
		return "UPDATED";
		break;
	case SIGNALSTATE_HOLD:
		return "HOLD";
		break;
	case SIGNALSTATE_CALIBRATING:
		return "CALIBRATING";
		break;
	default:
		return "UNDEFINED";
		break;
	}
}
eros::signal IMUNodeProcess::convert_signal(IMUDriver::Signal signal)
{
	eros::signal new_sig;
	new_sig.tov = signal.tov;
	new_sig.type = signal.type;
	new_sig.value = signal.value;
	new_sig.status = signal.state;
	return new_sig;
}
