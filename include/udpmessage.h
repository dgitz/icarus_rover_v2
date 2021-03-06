/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2020-01-25 07:38:19.291197***/
#ifndef UDPMESSAGE_H
#define UDPMESSAGE_H
#include "Definitions.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>


class UDPMessageHandler
{
public:
	enum MessageID
	{
		UDP_Command_ID = 0xAB02,
		UDP_RemoteControl_ID = 0xAB10,
		UDP_Resource_ID = 0xAB11,
		UDP_Diagnostic_ID = 0xAB12,
		UDP_Device_ID = 0xAB13,
		UDP_ArmControl_ID = 0xAB26,
		UDP_Arm_Status_ID = 0xAB30,
		UDP_Heartbeat_ID = 0xAB31,
		UDP_FindTarget_ID = 0xAB34,
		UDP_Power_ID = 0xAB37,
		UDP_EStop_ID = 0xAB38,
		UDP_TuneControlGroup_ID = 0xAB39,
		UDP_Firmware_ID = 0xAB41,
		UDP_SubsystemDiagnostic_ID = 0xAB43,
		UDP_SystemSnapshotState_ID = 0xAB44,
		UDP_ControlGroupValue_ID = 0xAB45,
		UDP_SystemState_ID = 0xAB46,
	};
	UDPMessageHandler();
	~UDPMessageHandler();
	int decode_CommandUDP(std::vector<std::string> items,uint8_t* Command,uint8_t* Option1,uint8_t* Option2,uint8_t* Option3,std::string* CommandText,std::string* Description);
	int decode_RemoteControlUDP(std::vector<std::string> items,uint64_t* Current_Timestamp,int* axis1,int* axis2,int* axis3,int* axis4,int* axis5,int* axis6,int* axis7,int* axis8,uint8_t* button1,uint8_t* button2,uint8_t* button3,uint8_t* button4,uint8_t* button5,uint8_t* button6,uint8_t* button7,uint8_t* button8);
	std::string encode_ResourceUDP(std::string Node_Name,uint16_t RAM_Mb,uint8_t CPU_Used);
	std::string encode_DiagnosticUDP(std::string DeviceName,std::string Node_Name,uint8_t System,uint8_t SubSystem,uint8_t Component,uint8_t Diagnostic_Type,uint8_t Level,uint8_t Diagnostic_Message,std::string Description);
	std::string encode_DeviceUDP(std::string DeviceParent,std::string DeviceName,std::string DeviceType,std::string Architecture);
	int decode_ArmControlUDP(std::vector<std::string> items,uint8_t* device,int* axis1,int* axis2,int* axis3,int* axis4,int* axis5,int* axis6,uint8_t* button1,uint8_t* button2,uint8_t* button3,uint8_t* button4,uint8_t* button5,uint8_t* button6);
	std::string encode_Arm_StatusUDP(uint8_t Status);
	int decode_HeartbeatUDP(std::vector<std::string> items,std::string* Device,uint64_t* Current_Timestamp,uint64_t* Expected_Timestamp);
	int decode_FindTargetUDP(std::vector<std::string> items,std::string* SearchDevice);
	std::string encode_PowerUDP(std::string BatteryName,uint8_t PowerLevel,uint8_t PowerState,double Voltage,double Current);
	std::string encode_EStopUDP(std::string DeviceName,uint8_t State);
	int decode_EStopUDP(std::vector<std::string> items,std::string* DeviceName,uint8_t* State);
	int decode_TuneControlGroupUDP(std::vector<std::string> items,std::string* ControlGroupName,double* value1,double* value2,double* value3,int* maxvalue,int* minvalue,int* defaultvalue);
	std::string encode_FirmwareUDP(std::string NodeName,std::string Description,uint8_t MajorRelease,uint8_t MinorRelease,uint8_t BuildNumber);
	std::string encode_SubsystemDiagnosticUDP(uint8_t Electrical,uint8_t Software,uint8_t Communications,uint8_t Sensors,uint8_t Actuators,uint8_t Data_Storage,uint8_t Remote_Control,uint8_t Target_Acquisition,uint8_t Pose,uint8_t Timing,uint8_t System_Resource);
	std::string encode_SystemSnapshotStateUDP(std::string State,uint8_t PercentComplete,uint16_t SystemSnapshotCount,std::string SourceDevice,std::string SystemSnapshotPath);
	std::string encode_ControlGroupValueUDP(double tov,std::string ControlGroupName,double Command,double Sense,double Error,double Error_perc,double Output,double IntegralError,double DerivativeError,double P_Output,double I_Output,double D_Output);
	std::string encode_SystemStateUDP(uint8_t State,uint8_t Option1,uint8_t Option2,uint8_t Option3,std::string StateText,std::string Description);
private:
};
#endif