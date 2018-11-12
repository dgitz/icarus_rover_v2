/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-11-12 06:50:47.053405***/
#ifndef JSONMESSAGE_H
#define JSONMESSAGE_H
#include "Definitions.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"

class JSONMessageHandler
{
public:
	enum MessageID
	{
		JSON_Diagnostic_ID = 0xAB12,
		JSON_Device_ID = 0xAB13,
		JSON_Arm_Status_ID = 0xAB30,
	};
	JSONMessageHandler();
	~JSONMessageHandler();
	std::string encode_DiagnosticJSON(icarus_rover_v2::diagnostic diagnostic);
	std::string encode_DeviceJSON(std::vector<icarus_rover_v2::device> devicelist);
	std::string encode_Arm_StatusJSON(uint8_t armed_status);
private:
};
#endif