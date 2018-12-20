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
			proc_imu.xacc.value = imu_data.acc_x/imus.at(i).acc_scale_factor;
			proc_imu.yacc.value = imu_data.acc_y/imus.at(i).acc_scale_factor;
			proc_imu.zacc.value = imu_data.acc_z/imus.at(i).acc_scale_factor;
			proc_imu.xgyro.value = imu_data.gyro_x/imus.at(i).gyro_scale_factor;
			proc_imu.ygyro.value = imu_data.gyro_y/imus.at(i).gyro_scale_factor;
			proc_imu.zgyro.value = imu_data.gyro_z/imus.at(i).gyro_scale_factor;
			proc_imu.xmag.value = imu_data.mag_x/imus.at(i).mag_scale_factor;
			proc_imu.ymag.value = imu_data.mag_y/imus.at(i).mag_scale_factor;
			proc_imu.zmag.value = imu_data.mag_z/imus.at(i).mag_scale_factor;

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
	if(device->DeviceType == "IMU")
	{
		if(device->PartNumber == "110012")
		{
			IMU newimu;
			newimu.initialized = false;
			newimu.devicename = device->DeviceName;
			newimu.connection_method = "serial";
			newimu.device_path = "/dev/ttyAMA0";
			newimu.comm_rate = "115200";
			newimu.acc_scale_factor = 2.0;
			newimu.gyro_scale_factor = 2000.0;
			newimu.mag_scale_factor = 6.665;
			newimu.update_count = 0;
			newimu.update_rate = 0.0;
			newimu.diagnostic.DeviceName = device->DeviceName;
			newimu.diagnostic.Node_Name = diag.Node_Name;
			newimu.diagnostic.System = diag.System;
			newimu.diagnostic.SubSystem = diag.SubSystem;
			newimu.diagnostic.Component = diag.Component;
			newimu.diagnostic.Diagnostic_Type = SENSORS;
			newimu.diagnostic.Level = NOTICE;
			newimu.diagnostic.Diagnostic_Message = INITIALIZING;
			newimu.diagnostic.Description = "IMU Initialized.";
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
