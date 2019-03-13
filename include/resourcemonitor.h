#ifndef RESOURCEMONITOR_H
#define RESOURCEMONITOR_H

#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <eros/resource.h>
#include <eros/diagnostic.h>
#include <numeric>
#include <boost/algorithm/string/replace.hpp>

#define SHORTTERM_BUFFER_SIZE 10
#define LONGTERM_BUFFER_MINSIZE 30
#define LONGTERM_BUFFER_SIZE 100
using std::string;
using namespace std;
class ResourceMonitor
{
public:
    ResourceMonitor();
    ResourceMonitor(eros::diagnostic diag,std::string DeviceArchitecture,std::string HostName,std::string TaskName);
    void init(eros::diagnostic diag,std::string DeviceArchitecture,std::string HostName,std::string TaskName);
    eros::diagnostic update();
    int get_CPUUsed_perc();  //Helper function, not needed in application.
    int get_RAMUsed_kB(); //Helper function, not needed in application.
    int get_TaskPID(); //Helper function, not needed in application.
    int get_CPUFree_perc();
    int get_RAMFree_kB();
    eros::resource get_resourceused();
    ~ResourceMonitor();
    std::string get_DeviceArchitecture() { return Device_Architecture; }
private:
    eros::diagnostic diagnostic;
    std::string Device_Architecture;
    std::string Task_Name;
    std::string Host_Name;
    std::string generic_node_name;  //Not specific to a Host
    int PID;
    int CPUUsed_perc;
    int RAMUsed_kB;
    int CPU_Used_Column;
    int RAM_Used_Column;
    int RAMFree_kB;
    int CPUFree_perc;

    std::vector<double> shortterm_buffer_RamUsed_kB;
    int shortterm_buffer_index;
    std::vector<double> longterm_buffer_RamUsed_kB;

};
#endif
