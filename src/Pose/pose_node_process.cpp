#include "pose_node_process.h"
PoseNodeProcess::PoseNodeProcess()
{
    pose_mode = UNDEFINED;
    wheelbase_m = 0.0;
    vehicle_length_m = 0.0;
    tirediameter_m = 0.0;
}   
PoseNodeProcess::~PoseNodeProcess()
{

}
icarus_rover_v2::diagnostic PoseNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
	initialized = false;
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
	beta = 0.0;
    initialized = true;
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
icarus_rover_v2::diagnostic PoseNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::pose temp_pose = pose;
	if(pose_mode == SIMULATED)
	{

		temp_pose.wheelspeed.value = throttle_command*tirediameter_m/2.0;
		double x_dot = temp_pose.wheelspeed.value * cos(pose.yaw.value);
		double y_dot = temp_pose.wheelspeed.value * sin(pose.yaw.value);
		//double psi_dot = (temp_pose.wheelspeed.value/l_f)*sin(beta);
		//beta = atan((l_f/vehicle_length_m)*tan(steer_command));
		temp_pose.east.value += (x_dot*dt);
		temp_pose.north.value += (y_dot*dt);
		temp_pose.yaw.value += (steer_command*dt);
		temp_pose.yaw.value = fmod(temp_pose.yaw.value + M_PI,2*M_PI);
		if(temp_pose.yaw.value < 0)
		{
			temp_pose.yaw.value += 2*M_PI;
		}
		temp_pose.yaw.value -= M_PI;
		pose_ready = true;

	}
	else if(pose_mode == REAL)
	{
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
		TiXmlElement *l_pDimensions = l_pRootElement->FirstChildElement( "Dimensions" );
		if ( NULL != l_pDimensions )
		{
			TiXmlElement *l_pWheelbase = l_pDimensions->FirstChildElement( "Wheelbase" );
			if(NULL != l_pWheelbase)
			{
				wheelbase_m = std::atof(l_pWheelbase->GetText());
			}
			else
			{
				printf("Wheelbase undefined.\n");
				return false;
			}

			TiXmlElement *l_pTireDiameter = l_pDimensions->FirstChildElement( "TireDiameter" );
			if(NULL != l_pTireDiameter)
			{
				tirediameter_m = std::atof(l_pTireDiameter->GetText());
			}
			else
			{
				printf("TireDiameter undefind.\n");
				return false;
			}

			TiXmlElement *l_pLength = l_pDimensions->FirstChildElement( "Length" );
			if(NULL != l_pLength)
			{
				vehicle_length_m = std::atof(l_pLength->GetText());
			}
			else
			{
				printf("Length undefined.\n");
				return false;
			}


		}
	}
	return true;
}
