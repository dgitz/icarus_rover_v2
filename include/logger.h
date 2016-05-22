#ifndef LOGGER_H
#define LOGGER_H

#include "ros/ros.h"
#include <icarus_rover_v2/Definitions.h>
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>

using std::string;
using namespace std;
class Logger
{
public:
    Logger();
    Logger(std::string level, std::string name);
    ~Logger();
    void log_debug(std::string tempstr);
    void log_info(std::string tempstr);
    void log_notice(std::string tempstr);
    void log_warn(std::string tempstr);
    void log_error(std::string tempstr);
    void log_fatal(std::string tempstr);
private:
    int line_counter;
    int verbosity;
    ofstream log_file;
    char file_path[120];
    void print_log(int level,std::string tempstr);
    int get_verbosity_level(std::string level);
};
#endif
