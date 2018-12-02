#include "pose_node_process.h"
PoseNodeProcess::PoseNodeProcess()
{
	gps_updated = false;
	last_theta_dot = 0.0;
	sampling_method = TimeCompensate::SAMPLEANDHOLD;
	run_time = 0.0;
}   
PoseNodeProcess::~PoseNodeProcess()
{

}
icarus_rover_v2::diagnostic PoseNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname_)
{
	diagnostic = indiag;
	initialized = false;
	hostname = hostname_;
	sensor_count = 0;
	initialize_filters();
	if(load_configfiles() == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Diagnostic_Message = ERROR;
		char tempstr[512];
		sprintf(tempstr,"Unable to load config files.");
		diagnostic.Description = std::string(tempstr);
	}
	left_encoder = 0.0;
	right_encoder = 0.0;
	pose.east.value = 0.0;
	pose.north.value = 0.0;
	pose.elev.value = 0.0;
	pose.wheelspeed.value = 0.0;
	pose.east.status = SIGNALSTATE_INITIALIZING;
	pose.north.status = SIGNALSTATE_INITIALIZING;
	pose.elev.status = SIGNALSTATE_INITIALIZING;
	pose.wheelspeed.status = SIGNALSTATE_INITIALIZING;
	throttle_command = 0.0;
	steer_command = 0.0;
	yaw_received = false;
	pose.yaw.value = 0.0;
	pose.yaw.status = SIGNALSTATE_INITIALIZING;
	yawrate_received = false;
	pose.yawrate.value = 0.0;
	pose.yawrate.status = SIGNALSTATE_INITIALIZING;
	pose_ready = false;
	temp_yaw = 0.0;
	return diagnostic;
}
KalmanFilter PoseNodeProcess::get_kalmanfilter(std::string name)
{
	for(std::size_t i = 0; i < KalmanFilters.size(); i++)
	{
		if(KalmanFilters.at(i).name == name)
		{
			return KalmanFilters.at(i);
		}
	}
	KalmanFilter empty;
	empty.name = "";
	return empty;
}
int PoseNodeProcess::get_kalmanfilter_index(std::string name)
{
	for(std::size_t i = 0; i < KalmanFilters.size(); i++)
	{
		if(KalmanFilters.at(i).name == name)
		{
			return i;
		}
	}
	return -1;
}
KalmanFilter PoseNodeProcess::update_kalmanfilter(KalmanFilter kf)
{
	//Predict
	kf.xhat =kf.xhat;
	kf.P = kf.Phi*kf.P*kf.Phi.transpose() + kf.Q;
	if(kf.name == "Yaw") 
	{
		//cout << "[predict]: xhat: " << kf.xhat << std::endl << " P: " << std::endl << kf.P << std::endl << std::endl;
	}
	//Update
	MatrixXd G1(kf.measurement_count,kf.measurement_count);
	G1 = kf.C*kf.P*kf.C.transpose() + kf.R;
	kf.G = kf.P*kf.C.transpose()*G1.inverse();
	kf.xhat = kf.xhat + kf.G*(kf.z-(kf.C*kf.xhat));
	MatrixXd I;//
	I = MatrixXd::Identity(kf.output_count, kf.output_count);
	kf.P = (I-kf.G*kf.C)*kf.P;
	if(kf.name == "Yaw")
	{
		//cout << "[update]: G: " << kf.G << std::endl << " xhat: " << kf.xhat << std::endl << " P: " << kf.P << std::endl << " C: " << kf.C << std::endl << " R: " << kf.R << std::endl << std::endl;
	}
	return kf;
}
void PoseNodeProcess::set_gps(icarus_rover_v2::pose v)
{
	gps = v;
	gps_updated = true;
}
icarus_rover_v2::diagnostic PoseNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
	for(std::size_t i = 0; i < IMU_Raw.size(); i++)
	{
		IMU_Signals.at(i) = timecompensate.sample(IMU_Raw.at(i),IMU_Signals.at(i),run_time);
	}
	//printf("%d\n",IMU_Signals.at(0).status);
	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = INFO;
	char tempstr[512];
	sprintf(tempstr,"Process Updated");
	diag.Description = std::string(tempstr);
	diagnostic.Diagnostic_Message = NOERROR;
	return diag;
	/*
	if(compute_pose == false)
	{
		icarus_rover_v2::diagnostic diag = diagnostic;
		pose = simpose;
		pose_ready = true;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		char tempstr[512];
		sprintf(tempstr,"Simulated Pose Node updated");
		diag.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = NOERROR;
		return diag;
	}
	else
	{
		{	//Pose from Odometry only
			icarus_rover_v2::diagnostic diag = diagnostic;
			icarus_rover_v2::pose temp_pose = pose;
			if(gps_updated == true)
			{
				temp_pose.east.value = gps.east.value;
				temp_pose.north.value = gps.north.value;
				temp_pose.yaw.value += (last_theta_dot);
				gps_updated = false;
			}
			else
			{

				double d_left = (vehicle_params.tirediameter_m/2.0)*left_encoder;
				double d_right = (vehicle_params.tirediameter_m/2.0)*right_encoder;
				double d_center = (d_left+d_right)/2.0;
				double theta_dot = (d_right-d_left)/vehicle_params.wheelbase_m;
				last_theta_dot = theta_dot;


				double x_dot = d_center * cos(temp_pose.yaw.value);
				double y_dot = d_center * sin(temp_pose.yaw.value);

				temp_pose.east.value += (x_dot);
				temp_pose.north.value += (y_dot);
				temp_pose.yaw.value += (theta_dot);
				temp_pose.yaw.value = fmod(temp_pose.yaw.value + M_PI,2*M_PI);
				if(temp_pose.yaw.value < 0)	{	temp_pose.yaw.value += 2*M_PI;	}
				temp_pose.yaw.value -= M_PI;
			}

			pose = temp_pose;
			pose_ready = true;

			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = INFO;
			char tempstr[512];
			sprintf(tempstr,"Pose Node updated");
			diag.Description = std::string(tempstr);
			diagnostic.Diagnostic_Message = NOERROR;
			return diag;
		}
		{
			icarus_rover_v2::diagnostic diag = diagnostic;
			icarus_rover_v2::pose temp_pose = pose;

			for(std::size_t i = 0; i < KalmanFilters.size(); i++)
			{
				KalmanFilters.at(i).signal_status = SIGNALSTATE_HOLD;
			}
			KalmanFilter KF_yawrate;
			KalmanFilter KF_yaw;


			//KF Updates - Acceleration

			//KF Updates - Rates/Velocities
			KF_yawrate = get_kalmanfilter("Yawrate");
			KF_yawrate = update_kalmanfilter(KF_yawrate);
			KF_yawrate.signal_status = SIGNALSTATE_UPDATED;
			KalmanFilters.at(get_kalmanfilter_index("Yawrate")) = KF_yawrate;
			//KF Updates - Position/Orientation
			KF_yaw = get_kalmanfilter("Yaw");
			temp_yaw += get_kalmanfilter_output("Yawrate",0)*(5*dt);
			{
				MatrixXd z(KF_yaw.measurement_count,1);
				z(0,0) = get_kalmanfilter_output("Yawrate",0);
				z(1,0) = temp_yaw;
				KF_yaw.z = z;
			}
			KF_yaw = update_kalmanfilter(KF_yaw);
			KF_yaw.signal_status = SIGNALSTATE_UPDATED;
			KalmanFilters.at(get_kalmanfilter_index("Yaw")) = KF_yaw;
			temp_pose.yaw.value = get_kalmanfilter_output("Yaw",1);
			//printf("Setting: %f\n",get_kalmanfilter_output("Yaw",1));

			for(std::size_t i = 0; i < KalmanFilters.size(); i++)
			{
				if(KalmanFilters.at(i).signal_status == SIGNALSTATE_HOLD)
				{
					KalmanFilters.at(i) = update_kalmanfilter(KalmanFilters.at(i));
					KalmanFilters.at(i).signal_status = SIGNALSTATE_UPDATED;
				}
			}

			if(yaw_received == true) {  temp_pose.yaw.status = SIGNALSTATE_UPDATED;  }
			temp_pose.yawrate.value = get_kalmanfilter_output("Yawrate",0);
			if(yawrate_received == true) { temp_pose.yawrate.status = SIGNALSTATE_UPDATED;   }
			if((yaw_received == true) &&
					(yawrate_received == true))
			{
				pose_ready = true;
			}

			if(isnan(temp_pose.yawrate.value) == true)
			{
				temp_pose.yawrate.value = 0.0;
				temp_pose.yawrate.status = SIGNALSTATE_INVALID;
			}
			if(isnan(temp_pose.yaw.value) == true)
			{
				temp_pose.yaw.value = 0.0;
				temp_pose.yaw.status = SIGNALSTATE_INVALID;
			}
			pose = temp_pose;
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = INFO;
			char tempstr[512];
			sprintf(tempstr,"Pose Node updated");
			diag.Description = std::string(tempstr);
			diagnostic.Diagnostic_Message = NOERROR;
			return diag;
		}
	}
	*/
}
double PoseNodeProcess::get_kalmanfilter_output(std::string name, int index) //Only used for unit testing
{
	for(std::size_t i = 0; i < KalmanFilters.size(); i++)
	{
		if(KalmanFilters.at(i).name == name)
		{
			//if(index == 0) { printf("a1: %f\n",KalmanFilters.at(i).xhat(0,0)); }
			//if(index == 1) { printf("a2: %f\n",KalmanFilters.at(i).xhat(1,0)); }
			return KalmanFilters.at(i).xhat(index,0);
		}
	}
	return 0.0;
}
icarus_rover_v2::diagnostic PoseNodeProcess::set_kalmanfilter_properties(std::string name, int measurement_count, int output_count, MatrixXd C,MatrixXd Phi, MatrixXd Q, MatrixXd R)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < KalmanFilters.size(); i++)
	{
		if(KalmanFilters.at(i).name == name)
		{
			//printf("name: %s v1: %d v2: %d\n",name.c_str(),measurement_count,output_count);
			KalmanFilters.at(i).measurement_count = measurement_count;
			KalmanFilters.at(i).output_count = output_count;
			KalmanFilters.at(i).xhat.resize(KalmanFilters.at(i).output_count,1);
			KalmanFilters.at(i).Phi.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
			KalmanFilters.at(i).z.resize(KalmanFilters.at(i).measurement_count,1);
			KalmanFilters.at(i).P.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
			KalmanFilters.at(i).P = MatrixXd::Ones(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
			KalmanFilters.at(i).Q.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
			KalmanFilters.at(i).G.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).measurement_count);
			KalmanFilters.at(i).C.resize(KalmanFilters.at(i).measurement_count,KalmanFilters.at(i).output_count);
			KalmanFilters.at(i).R.resize(KalmanFilters.at(i).measurement_count,KalmanFilters.at(i).measurement_count);
			KalmanFilters.at(i).z = MatrixXd::Zero(KalmanFilters.at(i).measurement_count,1);
			KalmanFilters.at(i).xhat = MatrixXd::Zero(KalmanFilters.at(i).output_count,1);
			KalmanFilters.at(i).Phi = Phi;
			KalmanFilters.at(i).Q = Q;
			KalmanFilters.at(i).R = R;
			KalmanFilters.at(i).C = C;
			KalmanFilters.at(i).signal_status = SIGNALSTATE_INITIALIZING;
			found = true;
		}
	}
	if(found == false)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = ERROR;
		char tempstr[512];
		sprintf(tempstr,"No Kalman Filter found with name: %s",name.c_str());
		diag.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	}
	else
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		char tempstr[512];
		sprintf(tempstr,"Set Kalman Filter: %s Properties.",name.c_str());
		diag.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = NOERROR;
	}
	return diag;
}
icarus_rover_v2::diagnostic PoseNodeProcess::new_kalmanfilter_signal(std::string name, int index, double v)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool found = false;
	for(std::size_t i = 0; i < KalmanFilters.size(); i++)
	{
		if(KalmanFilters.at(i).name == name)
		{

			if(name == "Yaw")
			{
				temp_yaw = v;
				yaw_received = true; 
			}
			else if(name == "Yawrate")
			{ 
				KalmanFilters.at(i).z(index,0) = v;
				yawrate_received = true; 
			}
			else
			{
				KalmanFilters.at(i).z(index,0) = v;
			}
			found = true;
		}
	}
	if(found == false)
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = ERROR;
		char tempstr[512];
		sprintf(tempstr,"No Kalman Filter found with name: %s",name.c_str());
		diag.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	}
	else
	{
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = INFO;
		char tempstr[512];
		sprintf(tempstr,"Kalman Filter found with name: %s",name.c_str());
		diag.Description = std::string(tempstr);
		diagnostic.Diagnostic_Message = NOERROR;
	}
	return diag;
}
void PoseNodeProcess::new_imudata(std::string sensorname,icarus_rover_v2::imu data)
{
	bool status = update_rawsignal("IMU",sensorname,data);
}
/*icarus_rover_v2::diagnostic PoseNodeProcess::new_kalmanfilter(std::string name, std::string type, int size)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    KalmanFilter KF;
    KF.name = name;
    KF.type = type;
    KF.Xk.resize(size,1);
    KF.Phi.resize(size,size);
    KF.Phi(0,0) = 1.0;
    KF.Phi(1,1) = 1.0;
    KF.P.resize(size,size);
    KF.Q.resize(size,size);
    KF.M.resize(size,1);
    KF.prev.resize(size,1);
    KF.R = 0.0;
    KF.output = 0.0;
    KF.input = 0.0;
    KalmanFilters.push_back(KF);

    diag.Diagnostic_Type = SOFTWARE;
    diag.Level = INFO;
    diag.Description = "Created new Kalman Filter";
    diagnostic.Diagnostic_Message = NOERROR;
    return diag;
}
 */
icarus_rover_v2::diagnostic PoseNodeProcess::new_kalmanfilter(std::string name, std::string type)
{
	icarus_rover_v2::diagnostic diag = diagnostic;

	KalmanFilter KF;
	KF.name = name;
	KF.type = type;
	KalmanFilters.push_back(KF);

	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = INFO;
	diag.Description = "Created new Kalman Filter";
	diagnostic.Diagnostic_Message = NOERROR;
	return diag;
}
void PoseNodeProcess::initialize_filters()
{
	{
		KalmanFilter KF;
		KF.name = "Yaw";
		KF.type = "Angle";
		KF.measurement_count = 2;
		KF.output_count = 2;
		KalmanFilters.push_back(KF);
	}

	{
		KalmanFilter KF;
		KF.name = "Yawrate";
		KF.type = "Rate";
		KF.measurement_count = 1;
		KF.output_count = 1;
		KalmanFilters.push_back(KF);
	}

	for(std::size_t i = 0; i < KalmanFilters.size(); i++)
	{
		KalmanFilters.at(i).signal_status = SIGNALSTATE_INITIALIZING;
		KalmanFilters.at(i).xhat.resize(KalmanFilters.at(i).output_count,1);
		KalmanFilters.at(i).Phi.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
		KalmanFilters.at(i).z.resize(KalmanFilters.at(i).measurement_count,1);
		KalmanFilters.at(i).P.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
		KalmanFilters.at(i).Q.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).output_count);
		KalmanFilters.at(i).G.resize(KalmanFilters.at(i).output_count,KalmanFilters.at(i).measurement_count);
		KalmanFilters.at(i).C.resize(KalmanFilters.at(i).measurement_count,KalmanFilters.at(i).output_count);
		KalmanFilters.at(i).R.resize(KalmanFilters.at(i).measurement_count,KalmanFilters.at(i).measurement_count);
	}
}
bool PoseNodeProcess::load_configfiles()
{
	TiXmlDocument miscconfig_doc("/home/robot/config/MiscConfig.xml");
	bool miscconfig_loaded = miscconfig_doc.LoadFile();
	if(miscconfig_loaded == false) { return false; }
	TiXmlElement *l_pRootElement = miscconfig_doc.RootElement();

	if( NULL != l_pRootElement )
	{
		TiXmlElement *l_pVehicleParameters = l_pRootElement->FirstChildElement( "VehicleParameters" );
		if ( NULL != l_pVehicleParameters )
		{
			TiXmlElement *l_pWheelbase = l_pVehicleParameters->FirstChildElement( "Wheelbase" );
			if(NULL != l_pWheelbase)
			{
				vehicle_params.wheelbase_m = std::atof(l_pWheelbase->GetText());
			}
			else
			{
				printf("Wheelbase undefined.\n");
				return false;
			}

			TiXmlElement *l_pTireDiameter = l_pVehicleParameters->FirstChildElement( "TireDiameter" );
			if(NULL != l_pTireDiameter)
			{
				vehicle_params.tirediameter_m = std::atof(l_pTireDiameter->GetText());
			}
			else
			{
				printf("TireDiameter undefined.\n");
				return false;
			}

			TiXmlElement *l_pLength = l_pVehicleParameters->FirstChildElement( "Length" );
			if(NULL != l_pLength)
			{
				vehicle_params.vehiclelength_m = std::atof(l_pLength->GetText());
			}
			else
			{
				printf("Length undefined.\n");
				return false;
			}

			TiXmlElement *l_pMaxSpeed = l_pVehicleParameters->FirstChildElement( "MaxSpeed" );
			if(NULL != l_pMaxSpeed)
			{
				vehicle_params.maxspeed_mps = std::atof(l_pMaxSpeed->GetText());
			}
			else
			{
				printf("MaxSpeed undefined.\n");
				return false;
			}


		}
		else
		{
			printf("VehicleParameters undefined. Exiting.\n");
			return false;
		}
	}
	return true;
}
icarus_rover_v2::diagnostic PoseNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == hostname)
	{
		sensor_count = device.SensorCount;
	}
	else
	{
		if((device.PartNumber == "110012")) //Supported IMU list
		{
			IMU_Sensor s;
			s.ready = false;
			s.name = device.DeviceName;
			s.id = device.ID;
			s.tov = 0.0;
			s.pn = device.PartNumber;
			s.initialized = true;
			imus.push_back(s);
		}
		else
		{
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			char tempstr[512];
			sprintf(tempstr,"%s Not Initialized",device.DeviceName.c_str());
			diag.Description = std::string(tempstr);
			diag.Diagnostic_Message = INITIALIZING_ERROR;
			return diag;
		}
	}
	if(sensor_count > 0)
	{
		if((imus.size() == sensor_count))
		{
			initialize_rawandextended_signals();
			timecompensate.set_samplingmethod(sampling_method);
			timecompensate.set_bufferlimit(500);
			initialized = true;
		}
	}
	return diag;
}
void PoseNodeProcess::initialize_rawandextended_signals()
{
	for(std::size_t i = 0; i < imus.size(); i++)
	{
		if(imus.at(i).pn == "110012")
		{
			Basic_Signal raw_imu_xacc;
			Basic_Signal raw_imu_yacc;
			Basic_Signal raw_imu_zacc;
			Basic_Signal raw_imu_xgyro;
			Basic_Signal raw_imu_ygyro;
			Basic_Signal raw_imu_zgyro;
			Basic_Signal raw_imu_xmag;
			Basic_Signal raw_imu_ymag;
			Basic_Signal raw_imu_zmag;

			{
				std::ostringstream ss1;
				ss1 << "xacc" << i+1;
				raw_imu_xacc.name = ss1.str();
				raw_imu_xacc.units = "meter/s^2";
				raw_imu_xacc.type = "Linear Acceleration";
				raw_imu_xacc.sensorsource = imus.at(i).pn;
				raw_imu_xacc.sensorname = imus.at(i).name;
				raw_imu_xacc.sensorindex = i+1;
				raw_imu_xacc.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "yacc" << i+1;
				raw_imu_yacc.name = ss1.str();
				raw_imu_yacc.units = "meter/s^2";
				raw_imu_yacc.type = "Linear Acceleration";
				raw_imu_yacc.sensorsource = imus.at(i).pn;
				raw_imu_yacc.sensorname = imus.at(i).name;
				raw_imu_yacc.sensorindex = i+1;
				raw_imu_yacc.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "zacc" << i+1;
				raw_imu_zacc.name = ss1.str();
				raw_imu_zacc.units = "meter/s^2";
				raw_imu_zacc.type = "Linear Acceleration";
				raw_imu_zacc.sensorsource = imus.at(i).pn;
				raw_imu_zacc.sensorname = imus.at(i).name;
				raw_imu_zacc.sensorindex = i+1;
				raw_imu_zacc.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "xgyro" << i+1;
				raw_imu_xgyro.name = ss1.str();
				raw_imu_xgyro.units = "deg/s";
				raw_imu_xgyro.type = "Angle Rate";
				raw_imu_xgyro.sensorsource = imus.at(i).pn;
				raw_imu_xgyro.sensorname = imus.at(i).name;
				raw_imu_xgyro.sensorindex = i+1;
				raw_imu_xgyro.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "ygyro" << i+1;
				raw_imu_ygyro.name = ss1.str();
				raw_imu_ygyro.units = "deg/s";
				raw_imu_ygyro.type = "Angle Rate";
				raw_imu_ygyro.sensorsource = imus.at(i).pn;
				raw_imu_ygyro.sensorname = imus.at(i).name;
				raw_imu_ygyro.sensorindex = i+1;
				raw_imu_ygyro.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "zgyro" << i+1;
				raw_imu_zgyro.name = ss1.str();
				raw_imu_zgyro.units = "deg/s";
				raw_imu_zgyro.type = "Angle Rate";
				raw_imu_zgyro.sensorsource = imus.at(i).pn;
				raw_imu_zgyro.sensorname = imus.at(i).name;
				raw_imu_zgyro.sensorindex = i+1;
				raw_imu_zgyro.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "xmag" << i+1;
				raw_imu_xmag.name = ss1.str();
				raw_imu_xmag.units = "uTesla";
				raw_imu_xmag.type = "Magnetic Field";
				raw_imu_xmag.sensorsource = imus.at(i).pn;
				raw_imu_xmag.sensorname = imus.at(i).name;
				raw_imu_xmag.sensorindex = i+1;
				raw_imu_xmag.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "ymag" << i+1;
				raw_imu_ymag.name = ss1.str();
				raw_imu_ymag.units = "uTesla";
				raw_imu_ymag.type = "Magnetic Field";
				raw_imu_ymag.sensorsource = imus.at(i).pn;
				raw_imu_ymag.sensorname = imus.at(i).name;
				raw_imu_ymag.sensorindex = i+1;
				raw_imu_ymag.computed_signal = 0;
			}
			{
				std::ostringstream ss1;
				ss1 << "zmag" << i+1;
				raw_imu_zmag.name = ss1.str();
				raw_imu_zmag.units = "uTesla";
				raw_imu_zmag.type = "Magnetic Field";
				raw_imu_zmag.sensorsource = imus.at(i).pn;
				raw_imu_zmag.sensorname = imus.at(i).name;
				raw_imu_zmag.sensorindex = i+1;
				raw_imu_zmag.computed_signal = 0;
			}

			IMU_Raw.push_back(raw_imu_xacc);
			IMU_Raw.push_back(raw_imu_yacc);
			IMU_Raw.push_back(raw_imu_zacc);
			IMU_Raw.push_back(raw_imu_xgyro);
			IMU_Raw.push_back(raw_imu_ygyro);
			IMU_Raw.push_back(raw_imu_zgyro);
			IMU_Raw.push_back(raw_imu_xmag);
			IMU_Raw.push_back(raw_imu_ymag);
			IMU_Raw.push_back(raw_imu_zmag);
		}
	}

	for(std::size_t i = 0; i < IMU_Raw.size(); i++)
	{
		Extended_Signal sig;
		sig.computed_signal = IMU_Raw.at(i).computed_signal;
		sig.name = IMU_Raw.at(i).name;
		sig.sensorindex = IMU_Raw.at(i).sensorindex;
		sig.sensorname = IMU_Raw.at(i).sensorname;
		sig.sensorsource = IMU_Raw.at(i).sensorsource;
		sig.type = IMU_Raw.at(i).type;
		sig.units = IMU_Raw.at(i).units;
		IMU_Signals.push_back(sig);
	}
}
std::vector<Basic_Signal> PoseNodeProcess::get_rawsignals(std::string sensortype)
{
	std::vector<Basic_Signal> empty;
	if(sensortype == "IMU")
	{
		return IMU_Raw;
	}
	return empty;
}
std::vector<Extended_Signal> PoseNodeProcess::get_extendedsignals(std::string sensortype)
{
	std::vector<Extended_Signal> empty;
	if(sensortype == "IMU")
	{
		return IMU_Signals;
	}
	return empty;
}
bool PoseNodeProcess::update_rawsignal(std::string sensortype,std::string sensorname,icarus_rover_v2::imu data)
{
	if(sensortype == "IMU")
	{
		for(std::size_t i = 0; i < IMU_Raw.size(); i++)
		{
			if(IMU_Raw.at(i).sensorname == sensorname)
			{
				std::size_t found_xacc = std::string(IMU_Raw.at(i).name).find("xacc");
				std::size_t found_yacc = std::string(IMU_Raw.at(i).name).find("yacc");
				std::size_t found_zacc = std::string(IMU_Raw.at(i).name).find("zacc");
				std::size_t found_xgyro = std::string(IMU_Raw.at(i).name).find("xgyro");
				std::size_t found_ygyro = std::string(IMU_Raw.at(i).name).find("ygyro");
				std::size_t found_zgyro = std::string(IMU_Raw.at(i).name).find("zgyro");
				std::size_t found_xmag = std::string(IMU_Raw.at(i).name).find("xmag");
				std::size_t found_ymag = std::string(IMU_Raw.at(i).name).find("ymag");
				std::size_t found_zmag = std::string(IMU_Raw.at(i).name).find("zmag");
				if(found_xacc != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.xacc.value;
					IMU_Raw.at(i).status = data.xacc.status;
					IMU_Raw.at(i).rms = data.xacc.rms;
				}
				else if(found_yacc != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.yacc.value;
					IMU_Raw.at(i).status = data.yacc.status;
					IMU_Raw.at(i).rms = data.yacc.rms;
				}
				else if(found_zacc != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.zacc.value;
					IMU_Raw.at(i).status = data.zacc.status;
					IMU_Raw.at(i).rms = data.zacc.rms;
				}
				else if(found_xgyro != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.xgyro.value;
					IMU_Raw.at(i).status = data.xgyro.status;
					IMU_Raw.at(i).rms = data.xgyro.rms;
				}
				else if(found_ygyro != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.ygyro.value;
					IMU_Raw.at(i).status = data.ygyro.status;
					IMU_Raw.at(i).rms = data.ygyro.rms;
				}
				else if(found_zgyro != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.zgyro.value;
					IMU_Raw.at(i).status = data.zgyro.status;
					IMU_Raw.at(i).rms = data.zgyro.rms;
				}
				else if(found_xmag != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.xmag.value;
					IMU_Raw.at(i).status = data.xmag.status;
					IMU_Raw.at(i).rms = data.xmag.rms;
				}
				else if(found_ymag != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.ymag.value;
					IMU_Raw.at(i).status = data.ymag.status;
					IMU_Raw.at(i).rms = data.ymag.rms;
				}
				else if(found_zmag != std::string::npos)
				{
					IMU_Raw.at(i).timestamp = data.tov;
					IMU_Raw.at(i).value = data.zmag.value;
					IMU_Raw.at(i).status = data.zmag.status;
					IMU_Raw.at(i).rms = data.zmag.rms;
				}
			}
		}
		return true;
	}
	else
	{
		return false;
	}

}
