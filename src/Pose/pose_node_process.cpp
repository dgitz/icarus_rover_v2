#include "pose_node_process.h"
PoseNodeProcess::PoseNodeProcess()
{
    initialized = false;
}
PoseNodeProcess::~PoseNodeProcess()
{

}
icarus_rover_v2::diagnostic PoseNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
    initialized = true;
	return diagnostic;
}
icarus_rover_v2::diagnostic PoseNodeProcess::update(double dt)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	return diag;
}
icarus_rover_v2::diagnostic PoseNodeProcess::set_kalmanfilter_properties(std::string name, double sigma_meas,double sigma_model)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        if(KalmanFilters.at(i).name == name)
        {
            KalmanFilters.at(i).sigma_meas = sigma_meas;
            KalmanFilters.at(i).sigma_model = sigma_model;
            /*
            KF_accy.Phi = [1 1/Pose_Rate; 0 1];
  KF_accy.type = 'Acceleration';
  KF_accy.sigma_model = 1;
  KF_accy.P = [KF_accy.sigma_model^2 0; 0 KF_accy.sigma_model^2];
  KF_accy.Q = [0 1; 0 0];
  KF_accy.M = [1 1];
  KF_accy.sigma_meas = 1;
  KF_accy.R = KF_accy.sigma_meas^2;
            */
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
icarus_rover_v2::diagnostic PoseNodeProcess::new_kalmanfilter_signal(std::string name, double input)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        if(KalmanFilters.at(i).name == name)
        {
            KalmanFilters.at(i).input = input;
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
icarus_rover_v2::diagnostic PoseNodeProcess::new_kalmanfilter(std::string name, std::string type, int size)
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
    KF.sigma_model = 0.0;
    KF.sigma_meas = 0.0;
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
void PoseNodeProcess::initialize_filters()
{
    {
        KalmanFilter KF;
        KF.name = "Yaw";
        KF.type = "Angle";
        KalmanFilters.push_back(KF);
    }
    
    {
        KalmanFilter KF;
        KF.name = "Yaw Rate";
        KF.type = "Rate";
        KalmanFilters.push_back(KF);
    }
    
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        KalmanFilters.at(i).Xk.resize(2,1);
        KalmanFilters.at(i).Phi.resize(2,2);
        KalmanFilters.at(i).P.resize(2,2);
        KalmanFilters.at(i).Q.resize(2,2);
        KalmanFilters.at(i).M.resize(2,1);
        KalmanFilters.at(i).prev.resize(2,1);
        KalmanFilters.at(i).sigma_model = 0.0;
        KalmanFilters.at(i).sigma_meas = 0.0;
        KalmanFilters.at(i).R = 0.0;
        KalmanFilters.at(i).output = 0.0;
        KalmanFilters.at(i).input = 0.0;
    }
}