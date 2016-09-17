/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-09-17 09:24:24.465156***/
/***Target: Raspberry Pi ***/
#include "serialmessage.h"
SerialMessageHandler::SerialMessageHandler(){}
SerialMessageHandler::~SerialMessageHandler(){}
int SerialMessageHandler::decode_DiagnosticSerial(unsigned char* inpacket,char* System,char* SubSystem,char* Component,char* Diagnostic_Type,char* Level,char* Diagnostic_Message)
{
	*System=inpacket[0];
	*SubSystem=inpacket[1];
	*Component=inpacket[2];
	*Diagnostic_Type=inpacket[3];
	*Level=inpacket[4];
	*Diagnostic_Message=inpacket[5];
	return 1;
}
int SerialMessageHandler::encode_TestMessageCounterSerial(unsigned char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x14;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = value1;
	*p_outbuffer++ = value2;
	*p_outbuffer++ = value3;
	*p_outbuffer++ = value4;
	*p_outbuffer++ = value5;
	*p_outbuffer++ = value6;
	*p_outbuffer++ = value7;
	*p_outbuffer++ = value8;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_TestMessageCounterSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8)
{
	*value1=inpacket[0];
	*value2=inpacket[1];
	*value3=inpacket[2];
	*value4=inpacket[3];
	*value5=inpacket[4];
	*value6=inpacket[5];
	*value7=inpacket[6];
	*value8=inpacket[7];
	return 1;
}
int SerialMessageHandler::encode_TestMessageCommandSerial(unsigned char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x15;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = value1;
	*p_outbuffer++ = value2;
	*p_outbuffer++ = value3;
	*p_outbuffer++ = value4;
	*p_outbuffer++ = value5;
	*p_outbuffer++ = value6;
	*p_outbuffer++ = value7;
	*p_outbuffer++ = value8;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::encode_Configure_DIO_PortASerial(unsigned char* outbuffer,int* length,char Pin1_Mode,char Pin2_Mode,char Pin3_Mode,char Pin4_Mode,char Pin5_Mode,char Pin6_Mode,char Pin7_Mode,char Pin8_Mode)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x16;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = Pin1_Mode;
	*p_outbuffer++ = Pin2_Mode;
	*p_outbuffer++ = Pin3_Mode;
	*p_outbuffer++ = Pin4_Mode;
	*p_outbuffer++ = Pin5_Mode;
	*p_outbuffer++ = Pin6_Mode;
	*p_outbuffer++ = Pin7_Mode;
	*p_outbuffer++ = Pin8_Mode;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::encode_GPIO_Board_ModeSerial(unsigned char* outbuffer,int* length,char Mode)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x17;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = Mode;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::encode_Set_DIO_PortASerial(unsigned char* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x18;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value;
	*p_outbuffer++ = Pin7_Value;
	*p_outbuffer++ = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Get_ANA_PortASerial(unsigned char* inpacket,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value)
{
	int v_Pin1_Value1=inpacket[0]<<8;
	*Pin1_Value=inpacket[1] + v_Pin1_Value1;
	int v_Pin2_Value1=inpacket[2]<<8;
	*Pin2_Value=inpacket[3] + v_Pin2_Value1;
	int v_Pin3_Value1=inpacket[4]<<8;
	*Pin3_Value=inpacket[5] + v_Pin3_Value1;
	int v_Pin4_Value1=inpacket[6]<<8;
	*Pin4_Value=inpacket[7] + v_Pin4_Value1;
	return 1;
}
int SerialMessageHandler::decode_Get_ANA_PortBSerial(unsigned char* inpacket,int* Pin1_Value,int* Pin2_Value,int* Pin3_Value,int* Pin4_Value)
{
	int v_Pin1_Value1=inpacket[0]<<8;
	*Pin1_Value=inpacket[1] + v_Pin1_Value1;
	int v_Pin2_Value1=inpacket[2]<<8;
	*Pin2_Value=inpacket[3] + v_Pin2_Value1;
	int v_Pin3_Value1=inpacket[4]<<8;
	*Pin3_Value=inpacket[5] + v_Pin3_Value1;
	int v_Pin4_Value1=inpacket[6]<<8;
	*Pin4_Value=inpacket[7] + v_Pin4_Value1;
	return 1;
}
int SerialMessageHandler::encode_Configure_DIO_PortBSerial(unsigned char* outbuffer,int* length,char Pin1_Mode,char Pin2_Mode,char Pin3_Mode,char Pin4_Mode,char Pin5_Mode,char Pin6_Mode,char Pin7_Mode,char Pin8_Mode)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x21;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = Pin1_Mode;
	*p_outbuffer++ = Pin2_Mode;
	*p_outbuffer++ = Pin3_Mode;
	*p_outbuffer++ = Pin4_Mode;
	*p_outbuffer++ = Pin5_Mode;
	*p_outbuffer++ = Pin6_Mode;
	*p_outbuffer++ = Pin7_Mode;
	*p_outbuffer++ = Pin8_Mode;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::encode_Set_DIO_PortBSerial(unsigned char* outbuffer,int* length,char Pin1_Value,char Pin2_Value,char Pin3_Value,char Pin4_Value,char Pin5_Value,char Pin6_Value,char Pin7_Value,char Pin8_Value)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = 0xAB;
	*p_outbuffer++ = 0x22;
	*p_outbuffer++ = 8;
	*p_outbuffer++ = Pin1_Value;
	*p_outbuffer++ = Pin2_Value;
	*p_outbuffer++ = Pin3_Value;
	*p_outbuffer++ = Pin4_Value;
	*p_outbuffer++ = Pin5_Value;
	*p_outbuffer++ = Pin6_Value;
	*p_outbuffer++ = Pin7_Value;
	*p_outbuffer++ = Pin8_Value;
	int checksum = 0;
	for(int i = 3; i < (3+8);i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	*length = p_outbuffer-&outbuffer[0];
	return 1;
}
int SerialMessageHandler::decode_Get_DIO_PortASerial(unsigned char* inpacket,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value)
{
	*Pin1_Value=inpacket[0];
	*Pin2_Value=inpacket[1];
	*Pin3_Value=inpacket[2];
	*Pin4_Value=inpacket[3];
	*Pin5_Value=inpacket[4];
	*Pin6_Value=inpacket[5];
	*Pin7_Value=inpacket[6];
	*Pin8_Value=inpacket[7];
	return 1;
}
int SerialMessageHandler::decode_Get_DIO_PortBSerial(unsigned char* inpacket,char* Pin1_Value,char* Pin2_Value,char* Pin3_Value,char* Pin4_Value,char* Pin5_Value,char* Pin6_Value,char* Pin7_Value,char* Pin8_Value)
{
	*Pin1_Value=inpacket[0];
	*Pin2_Value=inpacket[1];
	*Pin3_Value=inpacket[2];
	*Pin4_Value=inpacket[3];
	*Pin5_Value=inpacket[4];
	*Pin6_Value=inpacket[5];
	*Pin7_Value=inpacket[6];
	*Pin8_Value=inpacket[7];
	return 1;
}
