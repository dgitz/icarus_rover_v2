#ifndef GPIOHatDriver_h
#define GPIOHatDriver_h

#include <linux/i2c-dev.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), */
#include <sys/ioctl.h>

class GPIOHatDriver
{
public:


	GPIOHatDriver();
	~GPIOHatDriver();
	int init(int address = 0x14);
	int get_address() { return address; }
	int test_comm(int v);
	int i2cRead(char *data,int length);

	//Function: i2cWrite - First parameter is a pointer to a char array.containing the data to send.
	//...Second parameter is the length of the array.
	//...Returns: Error value. > 0 is ok. < 0 means there was an error.
	int i2cWrite(char *data,int length);

	//Function: i2cWrite - No Parameters.
	//...Returns: An Int value sent from the arduino, or -1 if there was an error.
	int i2cReadArduinoInt();

	//Function: i2cWrite - First parameter the int to send to the array.
	//...Returns: Error value. > 0 is ok. < 0 means there was an error.
	int i2cWriteArduinoInt(int input);
	int sendQuery(unsigned char query,unsigned char * inputbuffer);

private:
	int address;
	int GPIOHatfd;
};


#endif
