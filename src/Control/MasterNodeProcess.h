#ifndef MASTERNODEPROCESS_H
#define MASTERNODEPROCESS_H
#include "../include/Base/BaseNodeProcess.cpp"

//C System Files
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <iostream>
#include <fstream>

//C++ System Files
#include <boost/algorithm/string.hpp>

//ROS Base Functionality
#include "ros/ros.h"
#include "ros/time.h"

//ROS Messages
//Project
#include <math.h>
#include <tinyxml.h>
#include "../../include/serialmessage.h"
#include "Definitions.h"

/*! \class MasterNodeProcess MasterNodeProcess.h "MasterNodeProcess.h"
 *  \brief This is a MasterNodeProcess class.  Used for the master_node node.
 *
 */
class MasterNodeProcess: public BaseNodeProcess {
public:
    //Constants
	//Enums
	enum SerialPortType
	{
		UNDEFINED = 0,
		USB = 1,
		ACM =2,
		// SERIAL = 3
	};
	//Structs
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
	///Initialization Functions
	/*! \brief Set filepaths for DeviceFile and SystemFile */
	icarus_rover_v2::diagnostic set_filepaths(std::string t_system_filepath,std::string t_device_filepath);
	/*! \brief NodeProcess specific Initialization */
	icarus_rover_v2::diagnostic finish_initialization();
	/*! \brief Create Serial Port List */
	icarus_rover_v2::diagnostic set_serialportlist(std::vector<std::string> list);
	/*! \brief Create Active NodeList File. */
	bool create_nodelist(std::string nodelist_path,std::string activenode_path);
	/*! \brief Set Initialized, will also set Ready. */
	void set_initialized() { initialized = true; ready = true; }
	//Update Functions
	/*! \brief Implementation of the update function */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	std::vector<std::string> get_allserialbaudrates() { return serialport_baudrates; }
	std::vector<SerialPort> get_serialports() { return serialports; }
	std::vector<icarus_rover_v2::device> get_alldevices() { return allDevices; }
	std::vector<icarus_rover_v2::device> get_childdevices() { return childDevices; }
	std::vector<icarus_rover_v2::leverarm> get_allleverarms() { return leverarms; }
	bool get_leverarm(icarus_rover_v2::leverarm *leverarm,std::string name);
	double get_devicetemperature() { return device_temperature; }
	void set_devicetemperature(double t_temperature) { device_temperature = t_temperature; }
	//Message Functions
	/*! \brief  Process Command Message. */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
	/*! \brief Process new message received over Serial Port. */
	bool new_serialmessage(std::string serialport,std::string baudrate,unsigned char* message,int length); //Return true: valid, false: invalid
	//Support Functions

	//Printing Functions
	void print_device(std::vector<icarus_rover_v2::device> devices);
	void print_device(icarus_rover_v2::device device);
	void print_leverarm(std::vector<icarus_rover_v2::leverarm> leverarms);
	void print_leverarm(icarus_rover_v2::leverarm leverarm);
	void print_leverarm(std::string name,std::string reference,icarus_rover_v2::leverarm la);
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	bool build_childDevices();
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
	icarus_rover_v2::diagnostic load_devicefile(std::string path);
	icarus_rover_v2::diagnostic load_systemfile(std::string path);
	SerialMessageHandler *serialmessagehandler;

	std::vector<icarus_rover_v2::device> allDevices;
	std::vector<icarus_rover_v2::device> childDevices;
	std::vector<SerialPort> serialports;
	std::vector<std::string> serialport_baudrates;
	std::vector<icarus_rover_v2::leverarm> leverarms;
	std::string device_filepath;
	std::string system_filepath;
	double device_temperature;
	std::vector<icarus_rover_v2::device> devices_to_publish;
};
#endif
