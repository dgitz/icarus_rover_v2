/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-09-14 21:38:15.352220***/
#include "udpmessage.h"
UDPMessageHandler::UDPMessageHandler(){}
UDPMessageHandler::~UDPMessageHandler(){}
std::string UDPMessageHandler::encode_RemoteControlUDP(int16_t axis1,int16_t axis2)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_RemoteControl_ID));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)axis1));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)axis2));
	return tempstr;
}
std::string UDPMessageHandler::encode_ResourceUDP(std::string Node_Name,uint16_t RAM_Mb,uint8_t CPU_Used)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Resource_ID));
	tempstr.append(",");
	tempstr.append(Node_Name);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)RAM_Mb));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)CPU_Used));
	return tempstr;
}
std::string UDPMessageHandler::encode_DiagnosticUDP(std::string Node_Name,uint8_t System,uint8_t SubSystem,uint8_t Component,uint8_t Diagnostic_Type,uint8_t Level,uint8_t Diagnostic_Message,std::string Description)
{
	std::string tempstr = "";
	tempstr.append(boost::lexical_cast<std::string>(UDP_Diagnostic_ID));
	tempstr.append(",");
	tempstr.append(Node_Name);
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)System));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)SubSystem));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Component));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Diagnostic_Type));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Level));
	tempstr.append(",");
	tempstr.append(boost::lexical_cast<std::string>((int)Diagnostic_Message));
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
