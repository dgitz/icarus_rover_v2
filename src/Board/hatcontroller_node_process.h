#ifndef HATCONTROLLERNODEPROCESS_H
#define HATCONTROLLERNODEPROCESS_H

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
#include "icarus_rover_v2/iopins.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "logger.h"
#include <math.h>
#include <sys/time.h>
#include "ros/time.h"
#define TIMING_BUFFER_LENGTH 100
using std::string;
using namespace std;
class HatControllerNodeProcess
{
public:
	HatControllerNodeProcess();
	~HatControllerNodeProcess();    
    void init(std::string name,icarus_rover_v2::diagnostic diag);
    icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device newdevice);
    uint8_t get_armedstate() { return armed_state; }
    bool get_ready_to_arm() { return ready_to_arm; }
    bool is_initialized() { return initialized; }
    bool is_ready() { return ready; }
    void set_analyzetiming(bool v) { analyze_timing = v; }
    bool get_analyzetiming() { return analyze_timing; }
    double get_timedelay();
    icarus_rover_v2::diagnostic update(double dt);
    icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command msg);
    icarus_rover_v2::diagnostic new_armedstatemsg(uint8_t msg);
    icarus_rover_v2::diagnostic new_pinsmsg(icarus_rover_v2::iopins msg);
    icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin msg);
    icarus_rover_v2::diagnostic new_ppsmsg(std_msgs::Bool msg);
     
    //Servo Hat Functions
    icarus_rover_v2::diagnostic set_servohat_initialized(uint16_t id);
    std::vector<icarus_rover_v2::pin> get_servohatpins(uint16_t id);
    std::vector<uint16_t> get_servohataddresses();
    
    //Terminal Hat Functions
    icarus_rover_v2::diagnostic set_terminalhat_initialized();
    std::vector<icarus_rover_v2::pin> get_terminalhatpins(std::string Function);
    ros::Time convert_time(struct timeval t);
    
protected:
private:
    bool board_present(icarus_rover_v2::device device);
    double measure_time_diff(struct timeval start, struct timeval end);
    double measure_time_diff(ros::Time start, struct timeval end);
    double measure_time_diff(double start, struct timeval end);
    double measure_time_diff(double start, double end);
    double measure_time_diff(struct timeval start, double end);
    
    bool initialized;
    bool ready;
    std::string hostname;
    icarus_rover_v2::device mydevice;
    icarus_rover_v2::diagnostic diagnostic;
    uint8_t armed_state;
    bool ready_to_arm;
    std::vector<icarus_rover_v2::device> hats;
    std::vector<bool> hats_running;
    uint64_t pps_counter;
    double time_sincelast_pps;
    bool analyze_timing;
    std::vector<double> timing_diff;
};
#endif
