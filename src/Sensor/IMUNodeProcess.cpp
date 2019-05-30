#include "IMUNodeProcess.h"
eros::diagnostic  IMUNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	imus_initialized = false;
	imus_running = false;
	imu_reset_trigger = false;
	imu_reset_inprogress = false;
	imu_reset_inprogress_timer = 0.0;
	return diag;
}
eros::diagnostic IMUNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(initialized == true)
	{
		if(mydevice.SensorCount == 0)
		{
			ready = true;
		}
	}
	if((is_initialized() == true) and (is_ready() == true))
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
eros::diagnostic IMUNodeProcess::new_imumsg(std::string devicename,IMUDriver::RawIMU imu_data,eros::imu &proc_imu)
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{

			if((imu_data.signal_state == SIGNALSTATE_UPDATED) or 
			   (imu_data.signal_state == SIGNALSTATE_EXTRAPOLATED) or
			   (imu_data.signal_state == SIGNALSTATE_HOLD) 
			  )
			  {
				  imu_reset_inprogress = false;
				  imu_reset_inprogress_timer = 0.0;
			  }
			else
			{
				char tempstr[512];
				sprintf(tempstr,"IMU State: %s",map_signalstate_tostring(imu_data.signal_state).c_str());
				diag = update_diagnostic(devicename,SENSORS,WARN,DROPPING_PACKETS,std::string(tempstr));
				return diag;
			}
			proc_imu = imus.at(i).imu_data;
			imus.at(i).packet_count++;
			imus.at(i).sequence_number = imu_data.sequence_number;
			imus.at(i).imu_data.tov = imu_data.tov;
			proc_imu.tov = imu_data.tov;
			proc_imu.sequence_number = imu_data.sequence_number;
			{
				double x = imu_data.acc_x/imus.at(i).acc_scale_factor;
				double y = imu_data.acc_y/imus.at(i).acc_scale_factor;
				double z = imu_data.acc_z/imus.at(i).acc_scale_factor;
				Eigen::RowVector3f v;
				v << x,y,z;
				Eigen::RowVector3f vp = imus.at(i).rotate_matrix.Rotation_Acc*v.transpose();
				proc_imu.xacc.value = vp(0);
				proc_imu.yacc.value = vp(1);
				proc_imu.zacc.value = vp(2);
				if(imus.at(i).update_count == 0)
				{
					imus.at(i).xacc_rms_mean1 = pow(proc_imu.xacc.value,2.0);
				}
				else
				{
					imus.at(i).xacc_rms_mean1 += (pow(proc_imu.xacc.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).xacc_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.xacc.rms = pow(imus.at(i).xacc_rms_mean1,0.5);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).yacc_rms_mean1 = pow(proc_imu.yacc.value,2.0);
				}
				else
				{
					imus.at(i).yacc_rms_mean1 += (pow(proc_imu.yacc.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).yacc_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.yacc.rms = pow(imus.at(i).yacc_rms_mean1,0.5);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).zacc_rms_mean1 = pow(proc_imu.zacc.value,2.0);
				}
				else
				{
					imus.at(i).zacc_rms_mean1 += (pow(proc_imu.zacc.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).zacc_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.zacc.rms = pow(imus.at(i).zacc_rms_mean1,0.5);

			}

			{
				double x = imu_data.gyro_x/imus.at(i).gyro_scale_factor;
				double y = imu_data.gyro_y/imus.at(i).gyro_scale_factor;
				double z = imu_data.gyro_z/imus.at(i).gyro_scale_factor;
				Eigen::RowVector3f v;
				v << x,y,z;
				Eigen::RowVector3f vp = imus.at(i).rotate_matrix.Rotation_Gyro*v.transpose();
				proc_imu.xgyro.value = vp(0);
				proc_imu.ygyro.value = vp(1);
				proc_imu.zgyro.value = vp(2);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).xgyro_rms_mean1 = pow(proc_imu.xgyro.value,2.0);
				}
				else
				{
					imus.at(i).xgyro_rms_mean1 += (pow(proc_imu.xgyro.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).xgyro_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.xgyro.rms = pow(imus.at(i).xgyro_rms_mean1,0.5);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).ygyro_rms_mean1 = pow(proc_imu.ygyro.value,2.0);
				}
				else
				{
					imus.at(i).ygyro_rms_mean1 += (pow(proc_imu.ygyro.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).ygyro_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.ygyro.rms = pow(imus.at(i).ygyro_rms_mean1,0.5);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).zgyro_rms_mean1 = pow(proc_imu.zgyro.value,2.0);
				}
				else
				{
					imus.at(i).zgyro_rms_mean1 += (pow(proc_imu.zgyro.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).zgyro_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.zgyro.rms = pow(imus.at(i).zgyro_rms_mean1,0.5);

			}

			{
				double x = imu_data.mag_x/imus.at(i).mag_scale_factor;
				double y = imu_data.mag_y/imus.at(i).mag_scale_factor;
				double z = imu_data.mag_z/imus.at(i).mag_scale_factor;
				Eigen::RowVector3f v;
				v << x,y,z;
				Eigen::RowVector3f vp = imus.at(i).rotate_matrix.Rotation_Mag*v.transpose();
				proc_imu.xmag.value = vp(0);
				proc_imu.ymag.value = vp(1);
				proc_imu.zmag.value = vp(2);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).xmag_rms_mean1 = pow(proc_imu.xmag.value,2.0);
				}
				else
				{
					imus.at(i).xmag_rms_mean1 += (pow(proc_imu.xmag.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).xmag_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.xmag.rms = pow(imus.at(i).xmag_rms_mean1,0.5);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).ymag_rms_mean1 = pow(proc_imu.ymag.value,2.0);
				}
				else
				{
					imus.at(i).ymag_rms_mean1 += (pow(proc_imu.ymag.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).ymag_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.ymag.rms = pow(imus.at(i).ymag_rms_mean1,0.5);

				if(imus.at(i).update_count == 0)
				{
					imus.at(i).zmag_rms_mean1 = pow(proc_imu.zmag.value,2.0);
				}
				else
				{
					imus.at(i).zmag_rms_mean1 += (pow(proc_imu.zmag.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).zmag_rms_mean1/(double)(imus.at(i).update_count+1));
				}
				proc_imu.zmag.rms = pow(imus.at(i).zmag_rms_mean1,0.5);
			}


			proc_imu.xacc.status = imu_data.signal_state;
			proc_imu.yacc.status = imu_data.signal_state;
			proc_imu.zacc.status = imu_data.signal_state;
			proc_imu.xgyro.status = imu_data.signal_state;
			proc_imu.ygyro.status = imu_data.signal_state;
			proc_imu.zgyro.status = imu_data.signal_state;
			proc_imu.xmag.status = imu_data.signal_state;
			proc_imu.ymag.status = imu_data.signal_state;
			proc_imu.zmag.status = imu_data.signal_state;
			imus.at(i).lasttime_rx = run_time;
			imus.at(i).update_count++;
			imus.at(i).update_rate = (double)(imus.at(i).update_count)/run_time;
			imus.at(i).imu_data = proc_imu;
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
eros::diagnostic IMUNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	char tempstr[512];
	sprintf(tempstr,"DeviceType: %s Not Supported.",device->DeviceType.c_str());
	diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	return diag;
}
eros::diagnostic IMUNodeProcess::new_devicemsg(const eros::device::ConstPtr& device,const eros::leverarm::ConstPtr& leverarm)
{
	eros::diagnostic diag = root_diagnostic;
	if(device->DeviceType == "IMU")
	{
		if(device->PartNumber == "110012")
		{
			IMUNodeProcess::IMU newimu;
			newimu.initialized = false;
			newimu.devicename = device->DeviceName;
			newimu.connection_method = "serial";
			newimu.device_path = "/dev/ttyAMA0";
			newimu.comm_rate = "115200";
			newimu.partnumber = "110012";
			newimu.acc_scale_factor = 2000.0;
			newimu.gyro_scale_factor = 2000.0;
			newimu.mag_scale_factor = 6.665;

			newimu.update_count = 0;
			newimu.packet_count = 0;
			newimu.update_rate = 0.0;
			newimu.lasttime_rx = 0.0;
			newimu.sensor_info_path = "/home/robot/config/sensors/" + device->DeviceName + "/" + device->DeviceName + ".xml";
			newimu.diagnostic.DeviceName = device->DeviceName;
			newimu.diagnostic.Node_Name = diag.Node_Name;
			newimu.diagnostic.System = diag.System;
			newimu.diagnostic.SubSystem = diag.SubSystem;
			newimu.diagnostic.Component = diag.Component;
			newimu.diagnostic.Diagnostic_Type = SENSORS;
			newimu.diagnostic.Level = NOTICE;
			newimu.diagnostic.Diagnostic_Message = INITIALIZING;
			newimu.diagnostic.Description = "IMU Initialized.";
			newimu.imu_data.xacc.rms = 0.0;
			newimu.xacc_rms_mean1 = 0.0;
			newimu.mounting_angle_offset_pitch_deg = leverarm->pitch.value;
			newimu.mounting_angle_offset_roll_deg = leverarm->roll.value;
			newimu.mounting_angle_offset_yaw_deg = leverarm->yaw.value;
			newimu.rotate_matrix = generate_rotation_matrix(newimu.mounting_angle_offset_roll_deg,newimu.mounting_angle_offset_pitch_deg,newimu.mounting_angle_offset_yaw_deg);
			newimu.imu_data.xacc.units = "m/s^2";
			newimu.imu_data.yacc.units = "m/s^2";
			newimu.imu_data.zacc.units = "m/s^2";
			newimu.imu_data.xgyro.units = "deg/s";
			newimu.imu_data.ygyro.units = "deg/s";
			newimu.imu_data.zgyro.units = "deg/s";
			newimu.imu_data.xmag.units = "uT";
			newimu.imu_data.ymag.units = "uT";
			newimu.imu_data.zmag.units = "uT";
			newimu.initialized = true;
			imus.push_back(newimu);
			ready = true;
			imus_initialized = true;
			diag = update_diagnostic(SENSORS,INFO,NOERROR,"Sensors Initialized.");
			diag = update_diagnostic(newimu.diagnostic);
		}
		else if(device->PartNumber == "110015")
		{
			IMUNodeProcess::IMU newimu;
			newimu.initialized = false;
			newimu.devicename = device->DeviceName;
			newimu.connection_method = "serial";
			newimu.device_path = "/dev/ttyACM0";
			newimu.comm_rate = "115200";
			newimu.partnumber = "110015";
			newimu.acc_scale_factor = 1.0;
			newimu.gyro_scale_factor = 1.0;
			newimu.mag_scale_factor = 1.0;

			newimu.update_count = 0;
			newimu.packet_count = 0;
			newimu.update_rate = 0.0;
			newimu.lasttime_rx = 0.0;
			newimu.sensor_info_path = "/home/robot/config/sensors/" + device->DeviceName + "/" + device->DeviceName + ".xml";
			newimu.diagnostic.DeviceName = device->DeviceName;
			newimu.diagnostic.Node_Name = diag.Node_Name;
			newimu.diagnostic.System = diag.System;
			newimu.diagnostic.SubSystem = diag.SubSystem;
			newimu.diagnostic.Component = diag.Component;
			newimu.diagnostic.Diagnostic_Type = SENSORS;
			newimu.diagnostic.Level = NOTICE;
			newimu.diagnostic.Diagnostic_Message = INITIALIZING;
			newimu.diagnostic.Description = "IMU Initialized.";
			newimu.imu_data.xacc.rms = 0.0;
			newimu.xacc_rms_mean1 = 0.0;
			newimu.mounting_angle_offset_pitch_deg = leverarm->pitch.value;
			newimu.mounting_angle_offset_roll_deg = leverarm->roll.value;
			newimu.mounting_angle_offset_yaw_deg = leverarm->yaw.value;
			newimu.rotate_matrix = generate_rotation_matrix(newimu.mounting_angle_offset_roll_deg,newimu.mounting_angle_offset_pitch_deg,newimu.mounting_angle_offset_yaw_deg);
			newimu.imu_data.xacc.units = "m/s^2";
			newimu.imu_data.yacc.units = "m/s^2";
			newimu.imu_data.zacc.units = "m/s^2";
			newimu.imu_data.xgyro.units = "deg/s";
			newimu.imu_data.ygyro.units = "deg/s";
			newimu.imu_data.zgyro.units = "deg/s";
			newimu.imu_data.xmag.units = "uT";
			newimu.imu_data.ymag.units = "uT";
			newimu.imu_data.zmag.units = "uT";
			newimu.initialized = true;
			imus.push_back(newimu);
			ready = true;
			imus_initialized = true;
			diag = update_diagnostic(SENSORS,INFO,NOERROR,"Sensors Initialized.");
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
bool IMUNodeProcess::set_imu_info_path(std::string devicename,std::string path)
{
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			imus.at(i).sensor_info_path = path;
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
	R.Rotation_Acc_X.row(1) << 0.0,cos(roll),-sin(roll);
	R.Rotation_Acc_X.row(2) << 0.0,sin(roll),cos(roll);


	R.Rotation_Acc_Y.row(0) << cos(pitch),0.0,sin(pitch);
	R.Rotation_Acc_Y.row(1) << 0.0,1.0,0.0;
	R.Rotation_Acc_Y.row(2) << -sin(pitch),0.0,cos(pitch);

	R.Rotation_Acc_Z.row(0) << cos(yaw),-sin(yaw),0.0;
	R.Rotation_Acc_Z.row(1) << sin(yaw),cos(yaw),0.0;
	R.Rotation_Acc_Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Gyro_X.row(0) << 1.0,0.0,0.0;
	R.Rotation_Gyro_X.row(1) << 0.0,cos(roll),-sin(roll);
	R.Rotation_Gyro_X.row(2) << 0.0,sin(roll),cos(roll);

	R.Rotation_Gyro_Y.row(0) << cos(pitch),0.0,sin(pitch);
	R.Rotation_Gyro_Y.row(1) << 0.0,1.0,0.0;
	R.Rotation_Gyro_Y.row(2) << -sin(pitch),0.0,cos(pitch);

	R.Rotation_Gyro_Z.row(0) << cos(yaw),-sin(yaw),0.0;
	R.Rotation_Gyro_Z.row(1) << sin(yaw),cos(yaw),0.0;
	R.Rotation_Gyro_Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Mag_X.row(0) << 1.0,0.0,0.0;
	R.Rotation_Mag_X.row(1) << 0.0,cos(roll),-sin(roll);
	R.Rotation_Mag_X.row(2) << 0.0,sin(roll),cos(roll);

	R.Rotation_Mag_Y.row(0) << cos(pitch),0.0,sin(pitch);
	R.Rotation_Mag_Y.row(1) << 0.0,1.0,0.0;
	R.Rotation_Mag_Y.row(2) << -sin(pitch),0.0,cos(pitch);

	R.Rotation_Mag_Z.row(0) << cos(yaw),-sin(yaw),0.0;
	R.Rotation_Mag_Z.row(1) << sin(yaw),cos(yaw),0.0;
	R.Rotation_Mag_Z.row(2) << 0.0,0.0,1.0;

	R.Rotation_Acc = R.Rotation_Acc_Z*R.Rotation_Acc_Y*R.Rotation_Acc_X;
	R.Rotation_Gyro = R.Rotation_Gyro_Z*R.Rotation_Gyro_Y*R.Rotation_Gyro_X;
	R.Rotation_Mag = R.Rotation_Mag_Z*R.Rotation_Mag_Y*R.Rotation_Mag_X;
	return R;
}
/*! \brief Load Sensor Info from Config
 */
bool IMUNodeProcess::load_sensorinfo(std::string devicename)
{
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			TiXmlDocument doc(imus.at(i).sensor_info_path);
			bool loaded = doc.LoadFile();
			if(loaded == false)
			{
				printf("[%s]: Unable to load Sensor Config at: %s, but not required Yet.\n",devicename.c_str(),imus.at(i).sensor_info_path.c_str());
				return true;
			}
			TiXmlElement *l_pRootElement = doc.RootElement();
			double mao_roll,mao_pitch,mao_yaw;
			if(NULL != l_pRootElement)
			{
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
			else { return false;}
			return true;
		}
	}
	return false;
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
