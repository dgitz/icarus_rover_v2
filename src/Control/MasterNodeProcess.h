#ifndef MASTERNODEPROCESS_H
#define MASTERNODEPROCESS_H
#include "../../include/Base/BaseNodeProcess.cpp"

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
#include "../../include/Definitions.h"

#define TEMPERATURE_HIGH_VALUE 130.0
#define TEMPERATURE_LOW_VALUE 50.0

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
	eros::diagnostic set_filepaths(std::string t_system_filepath,std::string t_device_filepath);
	/*! \brief NodeProcess specific Initialization */
	eros::diagnostic finish_initialization();
	/*! \brief Create Serial Port List */
	eros::diagnostic set_serialportlist(std::vector<std::string> list);
	/*! \brief Create Active NodeList File. */
	bool create_nodelist(std::string nodelist_path,std::string activenode_path);
	/*! \brief Set Initialized, will also set Ready. */
	void set_initialized() { initialized = true; ready = true; }
	//Update Functions
	/*! \brief Implementation of the update function */
	eros::diagnostic update(double t_dt,double t_ros_time);
	eros::diagnostic slowupdate();
	eros::diagnostic process_loadfactormsg(std::string cmd);
	eros::diagnostic process_uptimemsg(std::string cmd);
	//Attribute Functions
	eros::loadfactor getLoadFactor() { return load_factor; }
	std_msgs::Float64 getUptime() 
	{
		std_msgs::Float64 v;
		v.data = uptime;
		return v; 
	}
	std::vector<std::string> get_allserialbaudrates() { return serialport_baudrates; }
	std::vector<SerialPort> get_serialports() { return serialports; }
	std::vector<eros::device> get_alldevices() { return allDevices; }
	std::vector<eros::device> get_childdevices() { return childDevices; }
	std::vector<eros::leverarm> get_allleverarms() { return leverarms; }
	bool get_leverarm(eros::leverarm *leverarm,std::string name);
	double get_devicetemperature() { return device_temperature; }
	void set_devicetemperature(double t_temperature)
	 { 
		 device_temperature = t_temperature; 
		 if(device_temperature> TEMPERATURE_HIGH_VALUE)
		{
			char tempstr[200];
			sprintf(tempstr,"Device Temperature: %4.2f",device_temperature);
			update_diagnostic(SENSORS,WARN,TEMPERATURE_HIGH,std::string(tempstr));
		}
		else if(device_temperature < TEMPERATURE_LOW_VALUE)
		{
		
			char tempstr[200];
			sprintf(tempstr,"Device Temperature: %4.2f",device_temperature);
			update_diagnostic(SENSORS,WARN,TEMPERATURE_LOW,std::string(tempstr));

		}
	}
	//Message Functions
	/*! \brief  Process Command Message. */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	/*! \brief Process new message received over Serial Port. */
	bool new_serialmessage(std::string serialport,std::string baudrate,unsigned char* message,int length); //Return true: valid, false: invalid
	//Support Functions

	//Printing Functions
	void print_device(std::vector<eros::device> devices);
	void print_device(eros::device device);
	void print_leverarm(std::vector<eros::leverarm> leverarms);
	void print_leverarm(eros::leverarm leverarm);
	void print_leverarm(std::string name,std::string reference,eros::leverarm la);
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	bool build_childDevices();
	std::vector<eros::diagnostic> check_programvariables();
	eros::diagnostic load_devicefile(std::string path);
	eros::diagnostic load_systemfile(std::string path);
	SerialMessageHandler *serialmessagehandler;

	std::vector<eros::device> allDevices;
	std::vector<eros::device> childDevices;
	std::vector<SerialPort> serialports;
	std::vector<std::string> serialport_baudrates;
	std::vector<eros::leverarm> leverarms;
	std::string device_filepath;
	std::string system_filepath;
	double device_temperature;
	std::vector<eros::device> devices_to_publish;
	eros::loadfactor load_factor;
	double uptime;
};
#endif
