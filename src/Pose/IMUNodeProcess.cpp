#include "IMUNodeProcess.h"
icarus_rover_v2::diagnostic  IMUNodeProcess::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	imus_initialized = false;
	imus_running = false;
	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		if(mydevice.SensorCount == 0)
		{
			ready = true;
		}
	}
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if((imus.at(i).running == true) and (run_time > 20.0))
		{
			if(imus.at(i).update_count == 0)
			{
				icarus_rover_v2::diagnostic imu_diagnostic = imus.at(i).diagnostic;
				imu_diagnostic.Diagnostic_Type = SENSORS;
				imu_diagnostic.Level = ERROR;
				imu_diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
				char tempstr[512];
				sprintf(tempstr,"Never Received data from IMU: %s",imus.at(i).devicename.c_str());
				imu_diagnostic.Description = std::string(tempstr);
				imus.at(i).diagnostic = imu_diagnostic;
				diag = imu_diagnostic;
			}
		}
	}
	if(diag.Level <= NOTICE)
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running.";

	}
	diagnostic = diag;
	return diag;
}
icarus_rover_v2::diagnostic IMUNodeProcess::new_imumsg(std::string devicename,IMUDriver::RawIMU imu_data,icarus_rover_v2::imu &proc_imu)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).devicename == devicename)
		{
			proc_imu = imus.at(i).imu_data;

			imus.at(i).imu_data.tov = imu_data.tov;
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
				imus.at(i).xacc_rms_mean1 += (pow(proc_imu.xacc.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).xacc_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.xacc.rms = pow(imus.at(i).xacc_rms_mean1,0.5);

				imus.at(i).yacc_rms_mean1 += (pow(proc_imu.yacc.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).yacc_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.yacc.rms = pow(imus.at(i).yacc_rms_mean1,0.5);

				imus.at(i).zacc_rms_mean1 += (pow(proc_imu.zacc.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).zacc_rms_mean1/(double)(imus.at(i).update_count+1));
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

				imus.at(i).xgyro_rms_mean1 += (pow(proc_imu.xgyro.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).xgyro_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.xgyro.rms = pow(imus.at(i).xgyro_rms_mean1,0.5);

				imus.at(i).ygyro_rms_mean1 += (pow(proc_imu.ygyro.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).ygyro_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.ygyro.rms = pow(imus.at(i).ygyro_rms_mean1,0.5);

				imus.at(i).zgyro_rms_mean1 += (pow(proc_imu.zgyro.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).zgyro_rms_mean1/(double)(imus.at(i).update_count+1));
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

				imus.at(i).xmag_rms_mean1 += (pow(proc_imu.xmag.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).xmag_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.xmag.rms = pow(imus.at(i).xmag_rms_mean1,0.5);

				imus.at(i).ymag_rms_mean1 += (pow(proc_imu.ymag.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).ymag_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.ymag.rms = pow(imus.at(i).ymag_rms_mean1,0.5);

				imus.at(i).zmag_rms_mean1 += (pow(proc_imu.zmag.value,2.0)/(double)((imus.at(i).update_count+1))) - (imus.at(i).zmag_rms_mean1/(double)(imus.at(i).update_count+1));
				proc_imu.zmag.rms = pow(imus.at(i).zmag_rms_mean1,0.5);
			}


			proc_imu.xacc.status = SIGNALSTATE_UPDATED;
			proc_imu.yacc.status = SIGNALSTATE_UPDATED;
			proc_imu.zacc.status = SIGNALSTATE_UPDATED;
			proc_imu.xgyro.status = SIGNALSTATE_UPDATED;
			proc_imu.ygyro.status = SIGNALSTATE_UPDATED;
			proc_imu.zgyro.status = SIGNALSTATE_UPDATED;
			proc_imu.xmag.status = SIGNALSTATE_UPDATED;
			proc_imu.ymag.status = SIGNALSTATE_UPDATED;
			proc_imu.zmag.status = SIGNALSTATE_UPDATED;

			imus.at(i).update_count++;
			imus.at(i).update_rate = (double)(imus.at(i).update_count)/run_time;
			imus.at(i).imu_data = proc_imu;
			found = true;
		}
	}
	if(found == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"Could not find Device: %s",devicename.c_str());
		diag.Description = std::string(tempstr);
	}
	else
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"Device: %s Updated",devicename.c_str());
		diag.Description = std::string(tempstr);
	}
	return diag;
}
icarus_rover_v2::diagnostic IMUNodeProcess::new_devicemsg(const icarus_rover_v2::device::ConstPtr& device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag.Diagnostic_Type = DATA_STORAGE;
	diag.Level = ERROR;
	diag.Diagnostic_Message = INITIALIZING_ERROR;
	char tempstr[512];
	sprintf(tempstr,"DeviceType: %s Not Supported.",device->DeviceType.c_str());
	diag.Description = std::string(tempstr);
	return diag;
}
icarus_rover_v2::diagnostic IMUNodeProcess::new_devicemsg(const icarus_rover_v2::device::ConstPtr& device,const icarus_rover_v2::leverarm::ConstPtr& leverarm)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
			newimu.acc_scale_factor = 2000.0;
			newimu.gyro_scale_factor = 2000.0;
			newimu.mag_scale_factor = 6.665;

			newimu.update_count = 0;
			newimu.update_rate = 0.0;
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
		}
		else
		{
			diag.Diagnostic_Type = DATA_STORAGE;
			diag.Level = ERROR;
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			char tempstr[512];
			sprintf(tempstr,"PartNumber: %s Not Supported.",device->PartNumber.c_str());
			diag.Description = std::string(tempstr);
		}
	}
	else
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		char tempstr[512];
		sprintf(tempstr,"DeviceType: %s Not Supported.",device->DeviceType.c_str());
		diag.Description = std::string(tempstr);
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
std::vector<icarus_rover_v2::diagnostic> IMUNodeProcess::new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
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
	return diaglist;
}
std::vector<icarus_rover_v2::diagnostic> IMUNodeProcess::check_programvariables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool status = true;

	if (status == true) {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED.";
		diaglist.push_back(diag);
	} else {
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED.";
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
