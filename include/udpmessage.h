/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-08-19 17:30:23.135270***/
#ifndef UDPMESSAGE_H
#define UDPMESSAGE_H
#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <iostream>

#define UDP_Resource_ID 0xAB11
#define UDP_Diagnostic_ID 0xAB12
#define UDP_Device_ID 0xAB13

class UDPMessageHandler
{
public:
	UDPMessageHandler();
	~UDPMessageHandler();
	std::string encode_ResourceUDP(std::string Node_Name,uint16_t RAM_Mb,uint8_t CPU_Used);
	std::string encode_DiagnosticUDP(std::string Node_Name,uint8_t System,uint8_t SubSystem,uint8_t Component,uint8_t Diagnostic_Type,uint8_t Level,uint8_t Diagnostic_Message,std::string Description);
	std::string encode_DeviceUDP(std::string DeviceParent,std::string DeviceName,std::string DeviceType,std::string Architecture);
private:
};
#endif