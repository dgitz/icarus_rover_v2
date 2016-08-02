/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-08-01 19:03:33.269182***/
#include "udpmessage.h"
UDPMessageHandler::UDPMessageHandler(){}
UDPMessageHandler::~UDPMessageHandler(){}
std::string UDPMessageHandler::encode_DiagnosticUDP(std::string Node_Name,uint8_t System,uint8_t SubSystem,uint8_t Component,uint8_t Diagnostic_Type,uint8_t Level,uint8_t Diagnostic_Message,std::string Description)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Diagnostic_ID));
	tempstr.append(",");
	tempstr.append(Node_Name);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>(System));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>(SubSystem));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>(Component));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>(Diagnostic_Type));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>(Level));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>(Diagnostic_Message));
	tempstr.append(",");
	tempstr.append(Description);
	return tempstr;
}
std::string UDPMessageHandler::encode_DeviceUDP(std::string DeviceParent,std::string DeviceName,std::string DeviceType,std::string Architecture)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Device_ID));
	tempstr.append(",");
	tempstr.append(DeviceParent);
	tempstr.append(",");
	tempstr.append(DeviceName);
	tempstr.append(",");
	tempstr.append(DeviceType);
	tempstr.append(",");
	tempstr.append(Architecture);
	return tempstr;
}
