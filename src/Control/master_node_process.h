#ifndef MASTERNODEPROCESS_H
#define MASTERNODEPROCESS_H

#include "ros/ros.h"
#include "ros/time.h"
#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include <boost/algorithm/string.hpp>
#include <math.h>
#include <tinyxml.h>
#include <iostream>
#include <fstream>
#include "../../include/serialmessage.h"
class MasterNodeProcess
{
public:

    enum SerialPortType
    {
        UNDEFINED = 0,
        USB = 1,
        ACM =2,
       // SERIAL = 3
    };
    struct SerialPort
    {
        bool available;
        bool checked;
        std::string file;
        uint8_t porttype;
        std::string baudrate;
        uint16_t id;
        std::string pn;
    };

	MasterNodeProcess();
	~MasterNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
    std::vector<icarus_rover_v2::device> get_alldevices() { return allDevices; }
    std::vector<icarus_rover_v2::device> get_childdevices() { return childDevices; }
	bool is_finished_initializing() { return all_device_info_received; }
    icarus_rover_v2::diagnostic load_devicefile(std::string path);
    void print_device(std::vector<icarus_rover_v2::device> devices);
    void print_device(icarus_rover_v2::device device);
    icarus_rover_v2::diagnostic set_serialportlist(std::vector<std::string> list);
    bool update_nodelist(std::string nodelist_path,std::string activenode_path);
    std::vector<std::string> get_allserialbaudrates() { return serialport_baudrates; }
    std::vector<SerialPort> get_serialports() { return serialports; }
    bool new_serialmessage(std::string serialport,std::string baudrate,unsigned char* message,int length); //Return true: valid, false: invalid
	
protected:

private:
    SerialMessageHandler *serialmessagehandler;
    bool build_childDevices();
	std::string myhostname;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
    std::vector<icarus_rover_v2::device> allDevices;
    std::vector<icarus_rover_v2::device> childDevices;
	icarus_rover_v2::diagnostic diagnostic;
    std::vector<SerialPort> serialports;
    std::vector<std::string> serialport_baudrates;
    
};
#endif
