#include "pose_node_process.h"
PoseNodeProcess::PoseNodeProcess()
{
    initialized = false;
    initialize_filters();
    yaw_received = false;
    pose.yaw.value = 0.0;
    pose.yaw.status = SIGNALSTATE_INITIALIZING;
    yawrate_received = false;
    pose.yawrate.value = 0.0;
    pose.yawrate.status = SIGNALSTATE_INITIALIZING;
    pose_ready = false;
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
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        KalmanFilters.at(i).Phi(0,0) = 1.0;
        KalmanFilters.at(i).Phi(0,1) = dt;
        KalmanFilters.at(i).Phi(1,0) = 0.0;
        KalmanFilters.at(i).Phi(1,1) = 1.0;
        
        //P1 = KF.Phi*KF.P*KF.Phi' + KF.Q;
        MatrixXd P1(2,2);
        P1 = KalmanFilters.at(i).Phi*KalmanFilters.at(i).P*KalmanFilters.at(i).Phi.transpose() + KalmanFilters.at(i).Q;
        
        //S = KF.M*P1*KF.M' + KF.R;
        MatrixXd S1(1,1);
        S1 = KalmanFilters.at(i).M.transpose()*P1*KalmanFilters.at(i).M; 
        double S = S1(0,0) + KalmanFilters.at(i).R;
        
        //KF.K = P1*KF.M'*inv(S);  
        MatrixXd K(2,1);
        K = P1*KalmanFilters.at(i).M*(1/S);
        
        //KF.P=P1-KF.K*KF.M*P1;
        MatrixXd temp1(1,1);
        temp1 = K.transpose()*KalmanFilters.at(i).M;
        KalmanFilters.at(i).P =  P1 - temp1(0,0)*P1;

        
        //KF.Xk = KF.Phi*KF.prev' + KF.K*(input-KF.M*KF.Phi*KF.prev');
        MatrixXd temp2(2,1);
        temp2 = KalmanFilters.at(i).Phi*KalmanFilters.at(i).prev;
        MatrixXd temp3(1,1);
        temp3 = KalmanFilters.at(i).M.transpose()*KalmanFilters.at(i).Phi*KalmanFilters.at(i).prev;
        KalmanFilters.at(i).Xk = temp2 + K*(KalmanFilters.at(i).input-temp3(0,0));
        KalmanFilters.at(i).prev = KalmanFilters.at(i).Xk;
        KalmanFilters.at(i).output = KalmanFilters.at(i).Xk(0,0);        
    }
    pose.yaw.value = get_kalmanfilter_output("Yaw");
    if(yaw_received == true) {  pose.yaw.status = SIGNALSTATE_UPDATED;  }
    pose.yawrate.value = get_kalmanfilter_output("Yawrate");
    if(yawrate_received == true) { pose.yawrate.status = SIGNALSTATE_UPDATED;   }
    
    if((yaw_received == true) &&
       (yawrate_received == true))
    {
       pose_ready = true;
    }
    diag.Diagnostic_Type = SOFTWARE;
    diag.Level = INFO;
    char tempstr[512];
    sprintf(tempstr,"Pose Node updated");
    diag.Description = std::string(tempstr);
    diagnostic.Diagnostic_Message = NOERROR;
    return diag;
}
double PoseNodeProcess::get_kalmanfilter_output(std::string name) //Only used for unit testing
{
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        if(KalmanFilters.at(i).name == name)
        {
            return KalmanFilters.at(i).output;
        }
    }
    return 0.0;
}
icarus_rover_v2::diagnostic PoseNodeProcess::set_kalmanfilter_properties(std::string name, double sigma_meas,double sigma_model)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        if(KalmanFilters.at(i).name == name)
        {
            KalmanFilters.at(i).P(0,0) = pow(sigma_model,2.0);
            KalmanFilters.at(i).P(0,1) = 0.0;
            KalmanFilters.at(i).P(1,0) = 0.0;
            KalmanFilters.at(i).P(1,1) = pow(sigma_model,2.0);
            KalmanFilters.at(i).R = pow(sigma_meas,2.0);
            KalmanFilters.at(i).Q(0,0) = 0.0;
            KalmanFilters.at(i).Q(0,1) = 1.0;
            KalmanFilters.at(i).Q(1,0) = 0.0;
            KalmanFilters.at(i).Q(1,1) = 0.0;
            KalmanFilters.at(i).M(0,0) = 1.0;
            KalmanFilters.at(i).M(1,0) = 1.0;
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
icarus_rover_v2::diagnostic PoseNodeProcess::new_kalmanfilter_signal(std::string name, double input)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
    bool found = false;
    for(std::size_t i = 0; i < KalmanFilters.size(); i++)
    {
        if(KalmanFilters.at(i).name == name)
        {
            KalmanFilters.at(i).input = input;
            if(name == "Yaw") { yaw_received = true; }
            else if(name == "Yawrate") { yawrate_received = true; }
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
        KF.name = "Yawrate";
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
        KalmanFilters.at(i).R = 0.0;
        KalmanFilters.at(i).output = 0.0;
        KalmanFilters.at(i).input = 0.0;
        KalmanFilters.at(i).prev(0,0) = 0.0;
        KalmanFilters.at(i).prev(1,0) = 0.0;
    }
}