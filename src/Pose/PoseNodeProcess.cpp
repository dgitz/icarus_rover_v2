#include "PoseNodeProcess.h"
icarus_rover_v2::diagnostic  PoseNodeProcess::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	imu_count = -1;
	current_mode = PoseMode::CALIBRATE;
	return diagnostic;
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
icarus_rover_v2::diagnostic PoseNodeProcess::update(double t_dt,double t_ros_time)
{
	if(initialized == true)
	{
		if((imu_count >= 1) and (imu_count == imus.size()))
		{
			ready = true;
		}
	}
	icarus_rover_v2::diagnostic diag = diagnostic;
	diag = update_baseprocess(t_dt,t_ros_time);
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
icarus_rover_v2::diagnostic PoseNodeProcess::new_devicemsg(const icarus_rover_v2::device::ConstPtr& device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
		newimu.initialized = true;
		imus.push_back(newimu);
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
std::vector<icarus_rover_v2::diagnostic> PoseNodeProcess::new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg)
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
std::vector<icarus_rover_v2::diagnostic> PoseNodeProcess::check_programvariables()
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
icarus_rover_v2::diagnostic PoseNodeProcess::new_imumsg(std::string topic, const icarus_rover_v2::imu::ConstPtr& data)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = WARN;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		char tempstr[512];
		sprintf(tempstr,"Received message for: %s But not in my Definitions.",topic.c_str());
		diag.Description = std::string(tempstr);
	}
	else
	{
		diag.Diagnostic_Type = NOERROR;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		char tempstr[512];
		sprintf(tempstr,"Received message for: %s.",topic.c_str());
		diag.Description = std::string(tempstr);
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
