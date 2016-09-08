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
#include <icarus_rover_v2/resource.h>

using std::string;
using namespace std;
class ResourceMonitor
{
public:
    ResourceMonitor();
    ResourceMonitor(std::string DeviceArchitecture,std::string HostName,std::string TaskName);
    bool update();
    int get_CPUUsed_perc();  //Helper function, not needed in application.
    int get_RAMUsed_kB(); //Helper function, not needed in application.
    int get_TaskPID(); //Helper function, not needed in application.
    int get_CPUFree_perc();
    int get_RAMFree_kB();
    icarus_rover_v2::resource get_resourceused();
    ~ResourceMonitor();
private:
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
};
#endif
