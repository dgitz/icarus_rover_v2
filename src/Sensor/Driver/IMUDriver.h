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
#define COMM_LOSS_THRESHOLD 1.0f
class IMUDriver
{
public:
    struct RawIMU
    {
        uint8_t signal_state;
        uint64_t update_count;
        double update_rate;
        double tov;
        double acc_x;
        double acc_y;
        double acc_z;
        double gyro_x;
        double gyro_y;
        double gyro_z;
        double mag_x;
        double mag_y;
        double mag_z;
    };

	IMUDriver();
	~IMUDriver();
	void set_debugmode(uint8_t v) { debug_mode = v; }
	int init(std::string t_connection_method,std::string t_port,std::string t_baudrate);
	int finish();
    
	RawIMU update();
    double measure_time_diff(struct timeval a,struct timeval b);
    double measure_time_diff(double a,double b);
    std::string map_signalstate_tostring(uint8_t v);
    std::string get_rawdata() { return raw_data; } //For debugging
    double get_timedelay() { return time_delay; }
private:
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
};


#endif