/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-08-12 21:01:44.417739***/
#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#define SERIAL_TestMessageCounter_ID 0x14
#define SERIAL_TestMessageCommand_ID 0x15

class SerialMessageHandler
{
public:
	SerialMessageHandler();
	~SerialMessageHandler();
	int encode_TestMessageCounterSerial(unsigned char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int decode_TestMessageCounterSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
	int encode_TestMessageCommandSerial(unsigned char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int decode_TestMessageCommandSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
private:
};
#endif