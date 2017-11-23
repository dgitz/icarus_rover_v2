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
		std::string pn;
		bool ready;
		bool data_available;
		int id;
		int seq;
		double tov;
		double last_tov;
		std::string name;
		double xacc;
		double xacc_rms;
        double xacc_lastsum;
		double yacc;
		double yacc_rms;
        double yacc_lastsum;
		double zacc;
		double zacc_rms;
        double zacc_lastsum;
		double xgyro;
		double xgyro_rms;
        double xgyro_lastsum;
		double ygyro;
		double ygyro_rms;
        double ygyro_lastsum;
		double zgyro;
		double zgyro_rms;
        double zgyro_lastsum;
		int xmag;
		double xmag_rms;
        double xmag_lastsum;
		int ymag;
		double ymag_rms;
        double ymag_lastsum;
		int zmag;
		double zmag_rms;
        double zmag_lastsum;
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
    
private:
	SerialMessageHandler *serialmessagehandler;
	icarus_rover_v2::device myDevice;
	icarus_rover_v2::diagnostic diagnostic;
	int sensor_count;
	Sensor sensor;
	bool running;
	int delay_counter;
	double run_time;
	bool message_ready;
	bool initialized;
};
#endif
