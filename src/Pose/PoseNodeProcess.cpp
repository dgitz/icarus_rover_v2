#include "PoseNodeProcess.h"
eros::diagnostic  PoseNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	imu_count = -1;
	current_mode = PoseMode::CALIBRATE;
	return diag;
}
bool PoseNodeProcess::set_imucount(uint8_t v)
{
	if(v >= 1)
	{
		imu_count = v;
		return true;
	}
	return false;
}
eros::diagnostic PoseNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if(initialized == true)
	{
		if((imu_count >= 1) and (imu_count == imus.size()))
		{
			ready = true;

		}
		else if(imu_count == 0)
		{
			diag = update_diagnostic(SENSORS,NOTICE,DEVICE_NOT_AVAILABLE,"No IMU's Configured.");
		}
	}
	diag = update_baseprocess(t_dt,t_ros_time);
	if((is_initialized() == true) and (is_ready() == true))
	{
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"No Error.");
	}
	if(diag.Level <= NOTICE)
	{
		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");
		diag = update_diagnostic(POSE,NOTICE,NOERROR,"Pose Not Implemented Yet.");

	}
	return diag;
}
PoseNodeProcess::IMUSensor PoseNodeProcess::get_imudata(std::string name)
{

	for(std::size_t i=0; i < imus.size(); ++i)
	{
		if(imus.at(i).device.DeviceName == name)
		{
			return imus.at(i);
		}
	}
	PoseNodeProcess::IMUSensor empty;
	return empty;
}
eros::diagnostic PoseNodeProcess::new_devicemsg(const eros::device::ConstPtr& device)
{
	eros::diagnostic diag = root_diagnostic;
	if(device->DeviceType == "IMU")
	{
		PoseNodeProcess::IMUSensor newimu;
		newimu.initialized = false;
		newimu.running = false;
		newimu.orientation_pitch.status = SIGNALSTATE_INITIALIZING;
		newimu.orientation_roll.status = SIGNALSTATE_INITIALIZING;
		newimu.orientation_yaw.status = SIGNALSTATE_INITIALIZING;
		newimu.orientation_pitch.units = "radians";
		newimu.orientation_roll.units = "radians";
		newimu.orientation_yaw.units = "radians";
		newimu.transform.setOrigin( tf::Vector3(0.0,0.0, 0.0) );
		tf::Quaternion q;
		q.setRPY(0.0,0.0,0.0);
		newimu.transform.setRotation(q);
		newimu.device = convert_fromptr(device);
		newimu.topicname = "/" + newimu.device.DeviceName;
		char tempstr[512];
		sprintf(tempstr,"Device: %s Initializing",newimu.device.DeviceName.c_str());
		diag = update_diagnostic(newimu.device.DeviceName,SENSORS,NOTICE,INITIALIZING,std::string(tempstr));
		diag = update_diagnostic(SENSORS,NOTICE,INITIALIZING,"Initializing IMU.");
		newimu.initialized = true;
		imus.push_back(newimu);
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"DeviceType: %s Not Supported.",device->DeviceType.c_str());
		diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
	}
	return diag;
}
std::vector<eros::diagnostic> PoseNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
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
	return diaglist;
}
std::vector<eros::diagnostic> PoseNodeProcess::check_programvariables()
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
eros::diagnostic PoseNodeProcess::new_imumsg(std::string topic, const eros::imu::ConstPtr& data)
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).topicname == topic)
		{
			imus.at(i).imu_data = convert_fromptr(data);
			imus.at(i).running = true;

			imus.at(i).orientation_roll.value = compute_acceleration_based_roll(imus.at(i).imu_data.xacc.value,
																			     imus.at(i).imu_data.yacc.value,
																				 imus.at(i).imu_data.zacc.value);
			imus.at(i).orientation_pitch.value = compute_acceleration_based_pitch(imus.at(i).imu_data.xacc.value,
				     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  imus.at(i).imu_data.yacc.value,
																				  imus.at(i).imu_data.zacc.value);
			imus.at(i).orientation_pitch.status = SIGNALSTATE_UPDATED;
			imus.at(i).orientation_roll.status = SIGNALSTATE_UPDATED;
			imus.at(i).orientation_yaw.value = 0.0;
			imus.at(i).transform.setOrigin( tf::Vector3(0.0,0.1+(double)(i)*0.1, 0.0) ); //Set Pose of IMU arbitrarily for debugging
			tf::Quaternion q;
			q.setRPY(-imus.at(i).orientation_roll.value, imus.at(i).orientation_pitch.value, imus.at(i).orientation_yaw.value);
			imus.at(i).transform.setRotation(q);
			found = true;
		}
	}
	if(found == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Received message for: %s But not in my Definitions.",topic.c_str());
		diag = update_diagnostic(DATA_STORAGE,WARN,DEVICE_NOT_AVAILABLE,std::string(tempstr));
	}
	else
	{
		char tempstr[512];
		sprintf(tempstr,"Received message for: %s.",topic.c_str());
		diag = update_diagnostic(SENSORS,INFO,NOERROR,std::string(tempstr));
	}
	return diag;

}
double PoseNodeProcess::compute_acceleration_based_roll(double xacc,double yacc,double zacc)
{
	double R = pow((xacc*xacc+yacc*yacc+zacc*zacc),0.5);
	return -asin(yacc/R);
}
double PoseNodeProcess::compute_acceleration_based_pitch(double xacc,double yacc,double zacc)
{
	double R = pow((xacc*xacc+yacc*yacc+zacc*zacc),0.5);
	return -asin(xacc/R);
}
std::string PoseNodeProcess::map_posemode_tostring(PoseNodeProcess::PoseMode t_posemode)
{
	switch(t_posemode)
	{
	case PoseMode::UNKNOWN:
		return "UNKNOWN";
	case PoseMode::CALIBRATE:
		return "CALIBRATE";
	default:
		return "UNKNOWN";
	}
}

PoseNodeProcess::PoseMode PoseNodeProcess::map_posemode_tovalue(std::string t_posemode)
{
	if(t_posemode == "UNKNOWN")
	{
		return PoseMode::UNKNOWN;
	}
	else if(t_posemode == "CALIBRATE")
	{
		return PoseMode::CALIBRATE;
	}
	else
	{
		return PoseMode::UNKNOWN;
	}
}
