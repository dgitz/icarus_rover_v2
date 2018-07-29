/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2018-07-29 08:09:41.999309***/
/***Target: Raspberry Pi ***/
#include "../include/spimessage.h"
SPIMessageHandler::SPIMessageHandler(){}
SPIMessageHandler::~SPIMessageHandler(){}
int SPIMessageHandler::decode_TestMessageCounterSPI(unsigned char* inbuffer,int * length,unsigned char* value1,unsigned char* value2,unsigned char* value3,unsigned char* value4,unsigned char* value5,unsigned char* value6,unsigned char* value7,unsigned char* value8,unsigned char* value9,unsigned char* value10,unsigned char* value11,unsigned char* value12)
{
	*value1 = inbuffer[0];
	*value2 = inbuffer[1];
	*value3 = inbuffer[2];
	*value4 = inbuffer[3];
	*value5 = inbuffer[4];
	*value6 = inbuffer[5];
	*value7 = inbuffer[6];
	*value8 = inbuffer[7];
	*value9 = inbuffer[8];
	*value10 = inbuffer[9];
	*value11 = inbuffer[10];
	*value12 = inbuffer[11];
	return 1;
}
int SPIMessageHandler::decode_Get_DIO_Port1SPI(unsigned char* inbuffer,int * length,uint16_t* EncoderA_TickSpeed_Offset,uint16_t* EncoderB_TickSpeed_Offset)
{
	int v_EncoderA_TickSpeed_Offset = inbuffer[0]<<8;
	*EncoderA_TickSpeed_Offset = v_EncoderA_TickSpeed_Offset + inbuffer[1];
	int v_EncoderB_TickSpeed_Offset = inbuffer[2]<<8;
	*EncoderB_TickSpeed_Offset = v_EncoderB_TickSpeed_Offset + inbuffer[3];
	return 1;
}
int SPIMessageHandler::decode_Get_ANA_Port1SPI(unsigned char* inbuffer,int * length,uint16_t* Pin1_Value,uint16_t* Pin2_Value,uint16_t* Pin3_Value,uint16_t* Pin4_Value,uint16_t* Pin5_Value,uint16_t* Pin6_Value)
{
	int v_Pin1_Value = inbuffer[0]<<8;
	*Pin1_Value = v_Pin1_Value + inbuffer[1];
	int v_Pin2_Value = inbuffer[2]<<8;
	*Pin2_Value = v_Pin2_Value + inbuffer[3];
	int v_Pin3_Value = inbuffer[4]<<8;
	*Pin3_Value = v_Pin3_Value + inbuffer[5];
	int v_Pin4_Value = inbuffer[6]<<8;
	*Pin4_Value = v_Pin4_Value + inbuffer[7];
	int v_Pin5_Value = inbuffer[8]<<8;
	*Pin5_Value = v_Pin5_Value + inbuffer[9];
	int v_Pin6_Value = inbuffer[10]<<8;
	*Pin6_Value = v_Pin6_Value + inbuffer[11];
	return 1;
}
int SPIMessageHandler::encode_LEDStripControlSPI(unsigned char* outbuffer,int * length,unsigned char LEDPixelMode,unsigned char Param1,unsigned char Param2)
{
	unsigned char *p_outbuffer;
	p_outbuffer = &outbuffer[0];
	*p_outbuffer++ = LEDPixelMode;
	*p_outbuffer++ = Param1;
	*p_outbuffer++ = Param2;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	*p_outbuffer++ = 0;
	unsigned char checksum = 0;
	for(int i = 0; i < 12; i++)
	{
		checksum ^= outbuffer[i];
	}
	*p_outbuffer++ = checksum;
	length[0] = 12;
	return 1;
}