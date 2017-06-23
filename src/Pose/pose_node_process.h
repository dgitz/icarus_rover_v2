#ifndef POSENODEPROCESS_H
#define POSENODEPROCESS_H

#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <Eigen/Dense>
#define BUFFER_LIMIT 100
using Eigen::MatrixXd;
struct KalmanFilter
{
    std::string name;
    std::string type;
    MatrixXd Xk; //(2,1)
    MatrixXd Phi; //(2,2)
    double sigma_model;
    MatrixXd P; //(2,2)
    MatrixXd Q; //(2,2)
    MatrixXd M; //(2,1)
    double sigma_meas;
    double R;
    std::vector<MatrixXd> Xk_buffer; //(vector(2,1)
    double output;
    double input;
    MatrixXd prev; //(2,1)
};
class PoseNodeProcess
{
public:


	PoseNodeProcess();
	~PoseNodeProcess();
    bool is_initialized() { return initialized; }
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
    icarus_rover_v2::diagnostic new_kalmanfilter(std::string name, std::string type, int size); //Only used for unit testing
    icarus_rover_v2::diagnostic new_kalmanfilter_signal(std::string name, double input);
    icarus_rover_v2::diagnostic set_kalmanfilter_properties(std::string name, double sigma_meas,double sigma_model);
    
private:
	icarus_rover_v2::diagnostic diagnostic;
    bool initialized;
    void initialize_filters();
    
    std::vector<KalmanFilter> KalmanFilters;
};
#endif
