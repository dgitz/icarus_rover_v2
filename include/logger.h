#ifndef LOGGER_H
#define LOGGER_H

#include "ros/ros.h"
#include "Definitions.h"
#include <eros/diagnostic.h>
#include "../../eROS/include/DiagnosticClass.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#define GREEN_FOREGROUND "\033[1;32m"
#define YELLOW_FOREGROUND "\033[1;33m"
#define RED_FOREGROUND "\033[1;31m"
#define END_COLOR "\033[0m"

using std::string;
using namespace std;
class Logger
{
public:
    Logger();
    Logger(std::string level, std::string name);
    Logger(std::string level,std::string modpath,std::string name);
    int get_logverbosity() { return verbosity; }
    ~Logger();

    void log_debug(std::string tempstr);
    void log_info(std::string tempstr);
    void log_notice(std::string tempstr);
    void log_warn(std::string tempstr);
    void log_error(std::string tempstr);
    void log_fatal(std::string tempstr);
    void log_diagnostic(eros::diagnostic diagnostic);
private:
    int line_counter;
    int verbosity;
    ofstream log_file;
    std::string node_name;
    char file_path[120];
    int get_verbosity_level(std::string level);
    void print_log(int level,std::string tempstr);

    DiagnosticClass diagclass;

};
#endif
