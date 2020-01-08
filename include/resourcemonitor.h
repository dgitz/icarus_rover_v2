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
#include <sys/resource.h>

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
    void disable_memoryleakdetection() { memoryleak_detectection_enabled = false; }
    eros::diagnostic update();
    int get_CPUUsed_perc();  //Helper function, not needed in application.
    int get_RAMUsed_kB(); //Helper function, not needed in application.
    int get_TaskPID(); //Helper function, not needed in application.
    int get_CPUFree_perc();
    int get_RAMFree_kB();
    int get_RAMFree_perc();
    int get_DISKFree_perc();
    eros::resource get_resourceused();
    ~ResourceMonitor();
    std::string get_DeviceArchitecture() { return Device_Architecture; }
private:
std::string exec(const char* cmd,bool wait_for_result);
    eros::diagnostic diagnostic;
    std::string Device_Architecture;
    std::string Task_Name;
    std::string Host_Name;
    std::string generic_node_name;  //Not specific to a Host
    int PID;
    int CPUUsed_perc;
    int RAMUsed_kB;
    int RAMFree_kB;
    int CPUFree_perc;
    int RAMFree_perc;
    uint64_t RAMTotal_kb;
    uint8_t processor_count;
    bool ramfree_initialized;
    bool memoryleak_detectection_enabled;

    std::vector<double> shortterm_buffer_RamUsed_kB;
    int shortterm_buffer_index;
    std::vector<double> longterm_buffer_RamUsed_kB;

};
#endif
