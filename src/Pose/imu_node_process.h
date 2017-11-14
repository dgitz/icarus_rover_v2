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
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include "icarus_rover_v2/imu.h"
#define RINGBUFFER_SIZE 500
class IMUNodeProcess
{

public:
	struct Sensor
	{
		bool ready;
		bool data_available;
		int id;
		double tov;
		double last_tov;
		std::string name;
		double xacc;
		boost::circular_buffer<double> xacc_buf;
		double xacc_rms;
		double yacc;
		boost::circular_buffer<double> yacc_buf;
		double yacc_rms;
		double zacc;
		boost::circular_buffer<double> zacc_buf;
		double zacc_rms;
		double xgyro;
		boost::circular_buffer<double> xgyro_buf;
		double xgyro_rms;
		double ygyro;
		boost::circular_buffer<double> ygyro_buf;
		double ygyro_rms;
		double zgyro;
		boost::circular_buffer<double> zgyro_buf;
		double zgyro_rms;
		int xmag;
		boost::circular_buffer<int> xmag_buf;
		double xmag_rms;
		int ymag;
		boost::circular_buffer<int> ymag_buf;
		double ymag_rms;
		int zmag;
		boost::circular_buffer<int> zmag_buf;
		double zmag_rms;
		int buffer_size;
	};

	IMUNodeProcess();
	~IMUNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	std::vector<Sensor> get_sensors() { return sensors; }
	void set_mydevice(icarus_rover_v2::device v) { myDevice = v; }
	icarus_rover_v2::device get_mydevice() { return myDevice; }
	bool new_message(std::string msg);
	bool imudata_ready(int id);
	icarus_rover_v2::imu get_imudata(int id);
	int get_delayedcounter() { return delay_counter; }
	double get_delayrate() { return (double)(delay_counter)/run_time; }
    
private:
	Sensor compute_otherimudata(Sensor sensor);
	double compute_rms(boost::circular_buffer<double> buf);
	double compute_rms(boost::circular_buffer<int> buf);
	icarus_rover_v2::device myDevice;
	icarus_rover_v2::diagnostic diagnostic;
	int sensor_count;
	std::vector<Sensor> sensors;
	bool running;
	int delay_counter;
	double run_time;
	bool message_ready;
};
#endif
