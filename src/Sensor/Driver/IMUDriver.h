/*TODO
	- Use auto-gen serial decode
    - convert imu msg to actual imu SI values
    - update rate is 1/2 (looks like update() returns empty message 1/2 of the time)

 */
#ifndef IMUtDriver_h
#define IMUDriver_h
#include "../../../include/eROS_Definitions.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <sys/time.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <sys/time.h>
#include "110015/AHRS.h"
#define G 9.807
#define COMM_LOSS_THRESHOLD 2.0f
class IMUDriver
{
public:
	enum PartNumber
	{
		UNKNOWN=0,
		PN_110013=1,
		PN_110015=2
	};
	struct Signal
	{
		double tov;
		uint8_t state;
		double value;
		uint8_t type;
	};
	struct RawIMU
	{
		uint64_t serial_number;
		bool updated;
		uint8_t signal_state;
		uint64_t update_count;

		double update_rate;
		double tov;
		uint16_t sequence_number;
		Signal acc_x;
		Signal acc_y;
		Signal acc_z;
		Signal gyro_x;
		Signal gyro_y;
		Signal gyro_z;
		Signal mag_x;
		Signal mag_y;
		Signal mag_z;
		Signal temperature;
	};

	IMUDriver();
	~IMUDriver();
	uint64_t get_serialnumber() { return imu_data.serial_number; }
	void set_debugmode(uint8_t v);
	int init(std::string t_partnumber,std::string t_port,std::string t_devicename,uint8_t verbosity);
	int finish();
	std::string get_port() { return port; }
	std::string map_pn_tostring(PartNumber v);
	IMUDriver::PartNumber map_pn_toenum(std::string v);
	RawIMU update();
	double measure_time_diff(struct timeval a,struct timeval b);
	double measure_time_diff(double a,double b);
	double convert_timestamp(struct timeval a);
	std::string map_signalstate_tostring(uint8_t v);
	std::string get_rawdata() { return raw_data; } //For debugging
	double get_timedelay() { return time_delay; }
	std::string get_devicename() { return devicename; }
	bool reset();

private:
	RawIMU init_imusignals();
	Signal init_signal(uint8_t type);
	Signal update_signal(double tov,uint16_t sequence_number,double value,Signal prev_signal);
	std::string devicename;
	PartNumber partnumber;
	uint16_t last_sequence_number;
	uint64_t convert_time_toms(struct timeval t);
	uint8_t debug_mode;
	std::vector<std::string> supported_connection_methods;
	std::string read_serialdata();
	double convert_time(struct timeval t);
	struct timeval now;
	struct timeval last;
	struct timeval last_timeupdate;
	bool timesync_tx_count;
	double time_delay;
	std::string connection_method;
	std::string port;
	std::string baudrate;
	int conn_fd;
	RawIMU imu_data;
	std::string raw_data;
	AHRS *device;

};


#endif
