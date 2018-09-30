#ifndef IMUNODEPROCESS_H
#define IMUNODEPROCESS_H

#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include "logger.h"
#include <math.h>
#include "icarus_rover_v2/imu.h"
#include <tinyxml.h>
#include <Eigen/Dense>
#define RINGBUFFER_SIZE 500
#define G  9.80665
using namespace Eigen;
class IMUNodeProcess
{

public:
	struct Sensor
	{
		std::string pn;
		bool ready;
		bool new_dataavailable;
		int id;
		int seq;
		double tov;
		double last_tov;
		double last_msgtime;
		std::string name;
		double xacc;
		double xacc_rms;
		std::vector<double> xacc_buffer;
        double xacc_lastsum;
        double xacc_bias;
		double yacc;
		double yacc_rms;
		std::vector<double> yacc_buffer;
        double yacc_lastsum;
        double yacc_bias;
		double zacc;
		double zacc_rms;
		std::vector<double> zacc_buffer;
        double zacc_lastsum;
        double zacc_bias;
		double xgyro;
		double xgyro_rms;
		std::vector<double> xgyro_buffer;
        double xgyro_lastsum;
        double xgyro_bias;
		double ygyro;
		double ygyro_rms;
		std::vector<double> ygyro_buffer;
        double ygyro_lastsum;
        double ygyro_bias;
		double zgyro;
		double zgyro_rms;
		std::vector<double> zgyro_buffer;
        double zgyro_lastsum;
        double zgyro_bias;
		double xmag;
		double xmag_rms;
		std::vector<double> xmag_buffer;
        double xmag_lastsum;
        double xmag_bias;
        double ymag;
		double ymag_rms;
		std::vector<double> ymag_buffer;
        double ymag_lastsum;
        double ymag_bias;
        double zmag;
		double zmag_rms;
		std::vector<double> zmag_buffer;
        double zmag_lastsum;
        double zmag_bias;
		int buffer_size;
	};

	IMUNodeProcess();
	~IMUNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	Sensor get_sensor() { return sensor; }
	void set_mydevice(icarus_rover_v2::device v) { myDevice = v; }
	icarus_rover_v2::device get_mydevice() { return myDevice; }
	bool new_message(std::string msg);
	bool new_serialmessage(unsigned char* message,int length); //Return true: valid, false: invalid
	//bool imudata_ready(int id);
	icarus_rover_v2::imu get_imudata(bool* valid);
	int get_delayedcounter() { return delay_counter; }
	double get_delayrate() { return run_time/(double)(delay_counter); }
	bool get_initialized() { return initialized; }
	void set_sensorname(std::string pn,std::string name,int id)
	{
		sensor.pn = pn;
		sensor.id = id;
		sensor.name = name;
	}
	std::string get_sensorname() { return sensor.name; }
    void set_verbositylevel(std::string v) { verbosity_level = v; }
    std::string get_verbositylevel() { return verbosity_level; }
    bool load_sensorinfo();
    bool set_mountingangle(double pitch_,double roll_,double yaw_);
    MatrixXd get_rotationmatrix() { return rotation_matrix; }
    
private:
    void update_rms();
    std::string hostname;
    std::string verbosity_level;
	SerialMessageHandler *serialmessagehandler;
	icarus_rover_v2::device myDevice;
	icarus_rover_v2::diagnostic diagnostic;
	Sensor sensor;
	bool running;
	int delay_counter;
	double run_time;
	bool message_ready;
	bool initialized;
    double pitchangle_rad;
    double rollangle_rad;
    double yawangle_rad;
    MatrixXd m_pitch;
    MatrixXd m_roll;
    MatrixXd m_yaw;
    MatrixXd rotation_matrix;
    int packet_missed_counter;
};
#endif
