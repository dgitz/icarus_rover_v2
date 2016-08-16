/***************AUTO-GENERATED.  DO NOT EDIT********************/
/***Created on:2016-08-15 18:02:44.558130***/
#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H
#define SERIAL_Diagnostic_ID 0x12
#define SERIAL_TestMessageCounter_ID 0x14
#define SERIAL_TestMessageCommand_ID 0x15

class SerialMessageHandler
{
public:
	SerialMessageHandler();
	~SerialMessageHandler();
	int encode_DiagnosticSerial(unsigned char* outbuffer,int* length,char System,char SubSystem,char Component,char Diagnostic_Type,char Level,char Diagnostic_Message);
	int decode_DiagnosticSerial(unsigned char* inpacket,char* System,char* SubSystem,char* Component,char* Diagnostic_Type,char* Level,char* Diagnostic_Message);
	int encode_TestMessageCounterSerial(unsigned char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int decode_TestMessageCounterSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
	int encode_TestMessageCommandSerial(unsigned char* outbuffer,int* length,char value1,char value2,char value3,char value4,char value5,char value6,char value7,char value8);
	int decode_TestMessageCommandSerial(unsigned char* inpacket,char* value1,char* value2,char* value3,char* value4,char* value5,char* value6,char* value7,char* value8);
private:
};
#endif