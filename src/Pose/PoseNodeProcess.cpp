#include "PoseNodeProcess.h"
eros::diagnostic  PoseNodeProcess::finish_initialization()
{
	expected_sensorsignal_count = 0;
	eros::diagnostic diag = root_diagnostic;
	imu_count = -1;
	current_mode = PoseMode::CALIBRATE;
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
	if((initialized == true) and (ready == false))
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
				/*
				std::vector<eros::signal> xacc_signals;
				std::vector<eros::signal> yacc_signals;
				std::vector<eros::signal> zacc_signals;
				for(std::size_t i = 0; i < imus.size(); ++i)
				{
					xacc_signals.push_back(imus.at(i).imu_data.xacc);
					yacc_signals.push_back(imus.at(i).imu_data.yacc);
					zacc_signals.push_back(imus.at(i).imu_data.zacc);
				}
				std::string linker_name;
				{	// XAcc Linker
					linker_name = "XAcc";
					xacc_linker = new LinearAccelerationLinker;
					if(xacc_linker->initialize_object(linker_name, diag) == false)
					{
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Unable to Initialize Linker: " + linker_name);
						return diag;
					}
					diag = update_diagnostic(xacc_linker->initialize_inputsignals(xacc_signals));
					if(diag.Level > ERROR)
					{
						return diag;
					}
					diag = update_diagnostic(xacc_linker->initialize_outputsignals(xacc_signals));
					if(diag.Level > ERROR)
					{
						return diag;
					}					
				}
				{	// YAcc Linker
					linker_name = "YAcc";
					yacc_linker = new LinearAccelerationLinker;
					if(xacc_linker->initialize_object(linker_name, diag) == false)
					{
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Unable to Initialize Linker: " + linker_name);
						return diag;
					}
					diag = update_diagnostic(yacc_linker->initialize_inputsignals(yacc_signals));
					if(diag.Level > ERROR)
					{
						return diag;
					}
					diag = update_diagnostic(yacc_linker->initialize_outputsignals(yacc_signals));
					if(diag.Level > ERROR)
					{
						return diag;
					}					
				}
				{	// ZAcc Linker
					linker_name = "ZAcc";
					zacc_linker = new LinearAccelerationLinker;
					if(zacc_linker->initialize_object(linker_name, diag) == false)
					{
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Unable to Initialize Linker: " + linker_name);
						return diag;
					}
					diag = update_diagnostic(zacc_linker->initialize_inputsignals(zacc_signals));
					if(diag.Level > ERROR)
					{
						return diag;
					}
					diag = update_diagnostic(zacc_linker->initialize_outputsignals(zacc_signals));
					if(diag.Level > ERROR)
					{
						return diag;
					}					
				}
				*/
				
			}
			diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"Node Ready.");
			diag = update_diagnostic(POSE,INFO,NOERROR,"Node Ready.");
			ready = true;
		}
	}
	diag = update_baseprocess(t_dt,t_ros_time);
	if((is_initialized() == true) and (is_ready() == true))
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
	if(device->DeviceType == "IMU")
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
			found = true;
			imus.at(i).imu_data = convert_fromptr(data);
			imus.at(i).running = true;
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.xacc,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.yacc,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.zacc,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.xgyro,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.ygyro,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.zgyro,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.xmag,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.ymag,"imu");
			update_sensorsignal(imus.at(i).imu_data.sequence_number,imus.at(i).imu_data.zmag,"imu");

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
eros::diagnostic PoseNodeProcess::update_pose(__attribute__((unused)) double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	if((sensor_signals.size() < expected_sensorsignal_count) || (expected_sensorsignal_count == 0))
	{
		diag = update_diagnostic(POSE,WARN,INITIALIZING,"Pose has not received all Sensor Inputs Yet.");
	}
	else
	{
		std::vector<TimedSignal> timedsignal_list;
		std::vector<PostProcessedSignal> postprocessedsignal_list;
		{ // Time Compensate
			
			for(std::size_t i = 0; i < sensor_signals.size(); ++i)
			{
				TimedSignal sig = time_compensators.at(i).new_signal(t_ros_time,sensor_signals.at(i));
				timedsignal_list.push_back(sig);
			}
		}
		{ // Sensor Post-Process
			for(std::size_t i = 0; i < timedsignal_list.size(); ++i)
			{
				for(std::size_t j = 0; j < imu_postprocessors.size(); ++j)
				{
					if(timedsignal_list.at(i).signal.name == imu_postprocessors.at(j).get_name())
					{
						PostProcessedSignal post_signal = imu_postprocessors.at(i).new_signal(timedsignal_list.at(i));
						postprocessedsignal_list.push_back(post_signal);
					}
				}
			}
		}
		{ // Signal Linkers
			//sensor_linearacceleration.xacc = xacc_linker->get_outputsignals();
			//sensor_linearacceleration.yacc = yacc_linker->get_outputsignals();
			//sensor_linearacceleration.zacc = zacc_linker->get_outputsignals();
		}
		diag = update_diagnostic(POSE,INFO,NOERROR,"Pose Updated at time: " + std::to_string(t_ros_time));
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
		SensorSignal new_sig;
		new_sig.sequence_number = sequence_number;
		new_sig.signal = signal;
		sensor_signals.push_back(new_sig);

		TimeCompensate tc;
		tc.init(new_sig.signal.name,TimeCompensate::SamplingMethod::SAMPLEANDHOLD);
		time_compensators.push_back(tc);

		if(source_sensor == "imu")
		{
			IMUPostProcess post_processor;
			post_processor.init(new_sig.signal.name);
			imu_postprocessors.push_back(post_processor);
		}
	}
}