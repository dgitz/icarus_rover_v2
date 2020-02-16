#include "PoseNodeProcess.h"
eros::diagnostic  PoseNodeProcess::finish_initialization()
{
	reset();
	expected_sensorsignal_count = 0;
	eros::diagnostic diag = root_diagnostic;
	imu_count = -1;
	current_mode = PoseMode::EXECUTE;
	return diag;
}
bool PoseNodeProcess::set_imucount(uint8_t v)
{
	if(v >= 1)
	{
		expected_sensorsignal_count+=9*v;
		imu_count = v;
		return true;
	}
	return false;
}
eros::diagnostic PoseNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).running == false)
		{
			diag = update_diagnostic(SENSORS,NOTICE,INITIALIZING,"No IMU Data Received yet for IMU: " + imus.at(i).topicname);
		}
	}
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
		bool checks_ok = true;
		if((imu_count >= 1) and (imu_count == imus.size()))
		{
			checks_ok = checks_ok && true;

		}
		else if(imu_count == 0)
		{
			checks_ok = false;
			diag = update_diagnostic(SENSORS,WARN,DEVICE_NOT_AVAILABLE,"No IMU's Configured.");
			return diag;
		}
		
		if(checks_ok == true)
		{
			// Ok to initialize objects
			{ // Time Compensate: Already initialized with Sensor_Signals
			}
			{ // Sensor Post Process
			}
			{ // Initialize Sensor Linkers
			
				
			}
			diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"Node Ready.");
			diag = update_diagnostic(POSE,INFO,NOERROR,"Node Ready.");
			request_statechange(TASKSTATE_RUNNING);
		}
	}
	else if(task_state == TASKSTATE_RUNNING)
	{
		if(current_mode == PoseMode::EXECUTE)
		{
		}
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
		diag = update_pose(t_dt,t_ros_time);
	}
	if(diag.Level <= NOTICE)
	{

		diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");


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
	if(device->DeviceType == DEVICETYPE_IMU)
	{
		PoseNodeProcess::IMUSensor newimu;
		newimu.initialized = false;
		newimu.running = false;
		newimu.orientation_pitch.status = SIGNALSTATE_INITIALIZING;
		newimu.orientation_roll.status = SIGNALSTATE_INITIALIZING;
		newimu.orientation_yaw.status = SIGNALSTATE_INITIALIZING;
		newimu.orientation_pitch.type = SIGNALTYPE_ANGLE;
		newimu.orientation_roll.type = SIGNALTYPE_ANGLE;
		newimu.orientation_yaw.type = SIGNALTYPE_ANGLE;
		/*newimu.transform.setOrigin( tf::Vector3(0.0,0.0, 0.0) );
		tf::Quaternion q;
		q.setRPY(0.0,0.0,0.0);
		newimu.transform.setRotation(q);
		*/
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
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
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
			found = true;
			imus.at(i).imu_data = convert_fromptr(data);
			imus.at(i).running = true;
			uint8_t index = map_imuname_toindex(imus.at(i).topicname);
			switch(index)
			{
				case 1:
					m_model.Pose_AutoCode_U.accel1x_in.value = data->xacc.value;
					m_model.Pose_AutoCode_U.accel1x_in.status = data->xacc.status;
					m_model.Pose_AutoCode_U.accel1x_in.rms = data->xacc.rms;
					m_model.Pose_AutoCode_U.accel1x_in.sequence_number = data->sequence_number;

					m_model.Pose_AutoCode_U.accel1y_in.value = data->yacc.value;
					m_model.Pose_AutoCode_U.accel1y_in.status = data->yacc.status;
					m_model.Pose_AutoCode_U.accel1y_in.rms = data->yacc.rms;
					m_model.Pose_AutoCode_U.accel1y_in.sequence_number = data->sequence_number;

					m_model.Pose_AutoCode_U.accel1z_in.value = data->zacc.value;
					m_model.Pose_AutoCode_U.accel1z_in.status = data->zacc.status;
					m_model.Pose_AutoCode_U.accel1z_in.rms = data->zacc.rms;
					m_model.Pose_AutoCode_U.accel1z_in.sequence_number = data->sequence_number;
					break;
				case 2:
					break;
				default:
					diag = update_diagnostic(DATA_STORAGE,WARN,DEVICE_NOT_AVAILABLE,"Unable to Resolve IMU: " + imus.at(i).topicname + " To Index.");
					return diag;
					break;
				//m_model.Pose_AutoCode_U.accel2x_in.value = data->xacc.value;
			}
			/*
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.xacc,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.yacc,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.zacc,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.xgyro,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.ygyro,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.zgyro,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.xmag,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.ymag,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.zmag,"imu");
			*/

			/*
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
			if((initialized == true) and (ready == true))
			{
				diag = update_diagnostic(xacc_linker->new_inputsignal(imus.at(i).imu_data.xacc));
				diag = update_diagnostic(yacc_linker->new_inputsignal(imus.at(i).imu_data.yacc));
				diag = update_diagnostic(zacc_linker->new_inputsignal(imus.at(i).imu_data.zacc));
			}
			*/
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
std::string PoseNodeProcess::map_posemode_tostring(PoseNodeProcess::PoseMode t_posemode)
{
	switch(t_posemode)
	{
	case PoseMode::UNKNOWN:
		return "UNKNOWN";
	case PoseMode::CALIBRATE:
		return "CALIBRATE";
	case PoseMode::EXECUTE:
		return "EXECUTE";
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
	else if(t_posemode == "EXECUTE")
	{
		return PoseMode::EXECUTE;
	}
	else
	{
		return PoseMode::UNKNOWN;
	}
}
eros::diagnostic PoseNodeProcess::update_pose(__attribute__((unused)) double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	bool ok_to_run = true;
	// Determine if all Sensor data is being received
	bool imus_ok = true;
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		if(imus.at(i).running == false)
		{
			imus_ok = false;
		}
	}
	if((imus_ok == true))
	{
		ok_to_run = true;
	}
	else
	{
		ok_to_run = false;
		diag = update_diagnostic(POSE,WARN,INITIALIZING,"Pose has not received all Sensor Inputs Yet.");
	}
	/*
	printf("check: %d %d\n",sensor_signals.size(),expected_sensorsignal_count);
	if((sensor_signals.size() < expected_sensorsignal_count) || (expected_sensorsignal_count == 0))
	{
		
	}
	else
	{	
		diag = update_diagnostic(POSE,INFO,NOERROR,"Pose Updated at time: " + std::to_string(t_ros_time));
	}
	*/
	if(ok_to_run == true)
	{
			m_model.Pose_AutoCode_U.current_time = run_time;
			m_model.step();
			pose_update_counter++;
			timed_signals.accel1x.value = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1X_INDEX].value;
			timed_signals.accel1x.status = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1X_INDEX].status;
			timed_signals.accel1x.rms = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1X_INDEX].rms;

			timed_signals.accel1y.value = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1Y_INDEX].value;
			timed_signals.accel1y.status = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1Y_INDEX].status;
			timed_signals.accel1y.rms = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1Y_INDEX].rms;

			timed_signals.accel1z.value = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1Z_INDEX].value;
			timed_signals.accel1z.status = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1Z_INDEX].status;
			timed_signals.accel1z.rms = m_model.Pose_AutoCode_Y.timed_signals_output[TIMEDSIGNAL_ACCEL1Z_INDEX].rms;
			diag = update_diagnostic(POSE,NOTICE,NOERROR,"Pose Last Updated at time: " + std::to_string(t_ros_time));

	
	}
	return diag;
}
void PoseNodeProcess::update_sensorsignal(uint64_t sequence_number,eros::signal signal,std::string source_sensor)
{
	bool found = false;
	for(std::size_t i = 0; i < sensor_signals.size(); ++i)
	{
		if(signal.name == sensor_signals.at(i).signal.name)
		{
			found = true;
			sensor_signals.at(i).sequence_number = sequence_number;
			sensor_signals.at(i).signal = signal;
		}
	}
	if(found == false)
	{
	}
}
uint8_t PoseNodeProcess::map_imuname_toindex(std::string name)
{
	uint8_t index = 0;
	// Search if imu is index:1
	if(name.find("1") != std::string::npos)
	{
		return 1;
	}
	else if(name.find("Left") != std::string::npos)
	{
		return 1;
	}
	// Search if imu is index:2
	else if(name.find("2") != std::string::npos)
	{
		return 2;
	}
	else if(name.find("Right") != std::string::npos)
	{
		return 2;
	}
	return index;
}