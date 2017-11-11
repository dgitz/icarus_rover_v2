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
#include "icarus_rover_v2/signal.h"
#include "icarus_rover_v2/pose.h"
#include "icarus_rover_v2/encoder.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <Eigen/Dense>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <tinyxml.h>
#include <tf/transform_broadcaster.h>
#define BUFFER_LIMIT 100
using Eigen::MatrixXd;
struct KalmanFilter
{
    std::string name;
    std::string type;
	uint8_t signal_status;
	int measurement_count; // m
	int output_count; // n
	MatrixXd z; //(m x 1)
    MatrixXd xhat; //(n x 1)
    MatrixXd Phi; // (n x n)
    MatrixXd G; // (n x m)
    MatrixXd Q; // (n x n)
    MatrixXd C; // (m x n)
    MatrixXd R; // (m x m)
	MatrixXd P; // (n x n)
};
class PoseNodeProcess
{

public:

	struct VehicleParameters
	{
		double tirediameter_m;
		double vehiclelength_m;
		double wheelbase_m;
		double maxspeed_mps;
	};

	PoseNodeProcess();
	~PoseNodeProcess();
    bool is_initialized() { return initialized; }
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
    icarus_rover_v2::diagnostic new_kalmanfilter(std::string name, std::string type); //Only used for unit testing
    double get_kalmanfilter_output(std::string name, int index); //Only used for unit testing
	KalmanFilter get_kalmanfilter(std::string name);
	int get_kalmanfilter_index(std::string name);
	KalmanFilter update_kalmanfilter(KalmanFilter kf);
    icarus_rover_v2::diagnostic new_kalmanfilter_signal(std::string name, int index, double v);
    icarus_rover_v2::diagnostic set_kalmanfilter_properties(std::string name, int measurement_count, int output_count, MatrixXd C,MatrixXd Phi, MatrixXd Q, MatrixXd R);
    icarus_rover_v2::pose get_pose() { return pose; }
    icarus_rover_v2::diagnostic new_yaw(double v);
    icarus_rover_v2::diagnostic new_yawrate(double v);
    bool is_poseready() { return pose_ready; }
    void set_throttlecommand(double v) { throttle_command = v; }
    void set_steercommand(double v) { steer_command = v; }
	void set_gps(icarus_rover_v2::pose v);
	void set_encoder(double v1,double v2) { left_encoder = v1; right_encoder = v2;}
    
    
private:
	icarus_rover_v2::diagnostic diagnostic;
    bool initialized;
    void initialize_filters();
    bool load_configfiles();
	VehicleParameters vehicle_params;
    
    std::vector<KalmanFilter> KalmanFilters;
    icarus_rover_v2::pose pose;
	icarus_rover_v2::pose gps;
    bool yaw_received;
    bool yawrate_received;
    bool pose_ready;
	double last_theta_dot;
    double throttle_command; //In rad/s
    double steer_command;  //In rad/s
	double left_encoder;
	double right_encoder;
	bool gps_updated;
	
	//temporary variables
	double temp_yaw;
};
#endif
