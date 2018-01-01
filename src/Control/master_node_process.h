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
#include "icarus_rover_v2/leverarm.h"
#include <boost/algorithm/string.hpp>
#include <math.h>
#include <tinyxml.h>
#include <iostream>
#include <fstream>
#include "../../include/serialmessage.h"
#define MAX_SERIALPACKET_SIZE 64
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
    struct LeverArm
    {
    	icarus_rover_v2::leverarm leverarm;
    	std::string name;
    	std::string reference;
    };

	MasterNodeProcess();
	~MasterNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname,std::string devicefilepath,std::string systemfilepath);
	icarus_rover_v2::diagnostic update(double dt);
	void set_diagnostic(icarus_rover_v2::diagnostic v) { diagnostic = v; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	double get_runtime() { return run_time; }
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	void set_mydevice(icarus_rover_v2::device device) { mydevice = device; initialized = true; }
	bool get_initialized() { return initialized; }
    bool get_ready() { return ready; }
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(icarus_rover_v2::command cmd);

	void set_initialized(bool v) { initialized = v; ready = true; }
    std::vector<icarus_rover_v2::device> get_alldevices() { return allDevices; }
    std::vector<icarus_rover_v2::device> get_childdevices() { return childDevices; }
    std::vector<LeverArm> get_allleverarms() { return leverarms; }

    void print_device(std::vector<icarus_rover_v2::device> devices);
    void print_device(icarus_rover_v2::device device);
    icarus_rover_v2::diagnostic set_serialportlist(std::vector<std::string> list);
    bool update_nodelist(std::string nodelist_path,std::string activenode_path);
    std::vector<std::string> get_allserialbaudrates() { return serialport_baudrates; }
    std::vector<SerialPort> get_serialports() { return serialports; }
    bool new_serialmessage(std::string serialport,std::string baudrate,unsigned char* message,int length); //Return true: valid, false: invalid
    void print_leverarm(std::vector<LeverArm> leverarms);
    void print_leverarm(LeverArm leverarm);
    void print_leverarm(std::string name,std::string reference,icarus_rover_v2::leverarm la);
    bool get_leverarm(icarus_rover_v2::leverarm *leverarm,std::string name);
   

	
protected:

private:
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();

    icarus_rover_v2::diagnostic load_devicefile(std::string path);
    icarus_rover_v2::diagnostic load_systemfile(std::string path);

	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    bool ready;

    SerialMessageHandler *serialmessagehandler;
    bool build_childDevices();
    std::vector<icarus_rover_v2::device> allDevices;
    std::vector<icarus_rover_v2::device> childDevices;
    std::vector<SerialPort> serialports;
    std::vector<std::string> serialport_baudrates;
    std::vector<LeverArm> leverarms;

};
#endif
