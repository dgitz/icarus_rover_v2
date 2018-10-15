#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include "../Driver/IMUDriver.h"
using namespace std;

static void show_usage(std::string name)
{
	std::cerr << "Usage: Test SPI Comm between IMU(s) and Raspberry Pi. Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-d,--delay Delay in MicroSeconds between sending each message.  Default is 100000.\n"
			<< "\t-id1,--IMU1 IMU1 ID.  ID is the equivalent to Slave Select Channel.\n"
			<< "\t-id2,--IMU2 IMU2 ID (Optional). ID is the equivalent to Slave Select Channel.\n"
			//<< "\t\t [0] Get_Diagnostic (0xAB12)\n"
			//<< "\t\t [1] TestMessageCounter (0xAB14)\n"
			//<< "\t\t [2] Get_DIO_Port1 (0xAB19)\n"
			//<< "\t\t [3] Get_ANA_Port1 (0xAB20)\n"
			//<< "\t-c,--command Command Message.  Supported messages are:\n"
			//<< "\t\t [0] LED Strip Control (0xAB42)\n"
			//<< "\t\t\t [0-5] [LEDPixelMode]\n"
			<< std::endl;
}

/**********************************************************
Housekeeping variables
 ***********************************************************/
int results;
int fd;
int first_message_received;
long passed_checksum;
long failed_checksum;
IMUDriver* imu;

/**********************************************************
Declare Functions
 ***********************************************************/

int spiTxRx(unsigned char txDat);
int sendQuery(unsigned char query, unsigned char * inputbuffer);
int sendCommand(unsigned char command,unsigned char* outputbuffer);
double dt(struct timeval a, struct timeval b);

/**********************************************************
Main
 ***********************************************************/

int main(int argc, char* argv[])
{
	int id1 = -1;
	int id2 = -1;
	long loop_delay = 100000;
	if (argc < 2) {
		show_usage(argv[0]);
		return 1;
	}
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if ((arg == "-h") || (arg == "--help"))
		{
			show_usage(argv[0]);
			return 0;
		}
		else if ((arg == "-d") || (arg == "--delay"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				loop_delay = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
				i++;
			}
		}
		else if ((arg == "-id1") || (arg == "--IMU1"))
		{
			if (i + 1 < argc)
			{
				id1 = atoi(argv[i+1]);
				i++;
			}
		}
		else if ((arg == "-id2") || (arg == "--IMU2"))
		{
			if (i + 1 < argc)
			{
				id2 = atoi(argv[i+1]);
				i++;
			}
		}
		if((id1 == -1) and (id2 == -1))
		{
			printf("ERROR: Must Use at least 1 IMU.\n");
			show_usage(argv[0]);
			return 1;
		}
	}
    int fd1,fd2 = -1;
    /*
	if(id1 != -1)
    {
        fd1 = wiringPiSPISetup(id1,1000000);
        if(fd1 <= 0)
        {
            printf("ERROR: FD1: %d\n",fd1);
            return 0;
        }
    }
    if(id2 != -1)
    {
        fd2 = wiringPiSPISetup(id2,1000000);
        if(fd2 <= 0)
        {
            printf("ERROR: FD1: %d\n",fd2);
            return 0;
        }
    }
    */
	

}

