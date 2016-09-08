#ifndef RESOURCEMONITOR_H
#define RESOURCEMONITOR_H

#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>

using std::string;
using namespace std;
class ResourceMonitor
{
public:
    ResourceMonitor();
    ResourceMonitor(std::string level, std::string name);
    ~ResourceMonitor();
private:
};
#endif
