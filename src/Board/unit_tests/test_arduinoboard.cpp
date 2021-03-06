#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include "../../../include/spimessage.h"
#include "../../../include/Supported_PN.h"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
using namespace std;

static void show_usage()
{
	std::cerr  << "This program is used to test the operation of a directly connected ArduinoBoard.\n"
			<< "Currently supported Part Numbers:\n"
			<< "\tPN: " << PN_100005 << "\n"
			<< "Usage: Test SPI Comm between Raspberry Pi and Arduino. Options:\n"
			<< "\t-h,--help\tShow this help message\n"
			<< "\t-d,--delay\tDelay in MicroSeconds between sending each message.  Default is 100000.\n"
			<< "\t-q,--query\tQuery Message.  Supported messages are:\n"
			<< "\t\t [0] Get_Diagnostic (0xAB12)\n"
			<< "\t\t [1] TestMessageCounter (0xAB14)\n"
			<< "\t\t [2] Get_DIO_Port1 (0xAB19)\n"
			<< "\t\t [3] Get_ANA_Port1 (0xAB20)\n"
			<< "\t-c,--command\tCommand Message.  Supported messages are:\n"
			<< "\t\t [0] LED Strip Control (0xAB42)\n"
			<< "\t\t\t [0-5] [LEDPixelMode:0=None,1=Normal,2=Warn,3=Error,4=Color Cycle,5=Focus]\n"
			<< "\t\t [1] ArmStatus (0xAB30)\n"
			<< "\t\t\t [0-5] [Arm Status]\n"
			<< "\t\t [2] Command (0xAB02)\n"
			<< "\t\t\t [0-5] [Command]\n"
			<< "\t\t\t [0-5] [Option1]\n"
			<< "\t\t\t [0-5] [Option2]\n"
			<< "\t\t\t [0-5] [Option3]\n"
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
SPIMessageHandler *spimessagehandler;

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
	passed_checksum = 0;
	failed_checksum = 0;
	int query_message = 0;
	int command_message = 0;
	int param1 = 0;
	int param2 = 0;
	int param3 = 0;
	int param4 = 0;
	unsigned char query_type;
	unsigned char command_type;
	long loop_delay = 100000;
	if (argc < 2) {
		show_usage();
		return 1;
	}
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if ((arg == "-h") || (arg == "--help"))
		{
			show_usage();
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
		else if ((arg == "-q") || (arg == "--query"))
		{
			command_message = 0;
			query_message = 1;
			if (i + 1 < argc)
			{
				int v = atoi(argv[i+1]);
				switch(v)
				{
				case 0:
					query_type = SPIMessageHandler::SPI_Diagnostic_ID;
					break;
				case 1:
					query_type = SPIMessageHandler::SPI_TestMessageCounter_ID;
					break;
				case 2:
					query_type = SPIMessageHandler::SPI_Get_DIO_Port1_ID;
					break;
				case 3:
					query_type = SPIMessageHandler::SPI_Get_ANA_Port1_ID;
					break;
				default:
					printf("Unsupported Query Message.  Exiting.\n");
					return 0;
				}
				i++;
			}
		}
		else if ((arg == "-c") || (arg == "--command"))
		{
			command_message = 1;
			query_message = 0;
			if (i + 1 < argc)
			{
				int v = atoi(argv[i+1]);
				switch(v)
				{
				case 0:
					command_type = SPIMessageHandler::SPI_LEDStripControl_ID;
					param1 = atoi(argv[i+2]);
					break;
				case 1:
					command_type = SPIMessageHandler::SPI_Arm_Status_ID;
					param1 = atoi(argv[i+2]);
					break;
				case 2:
					command_type = SPIMessageHandler::SPI_Command_ID;
					param1 = atoi(argv[i+2]);
					param2 = atoi(argv[i+3]);
					param3 = atoi(argv[i+4]);
					param4 = atoi(argv[i+5]);
					break;
				default:
					printf("Unsupported Command Message.  Exiting.\n");
					return 0;
				}
				i++;
			}
		}

	}
	first_message_received = 0;
	fd = open("/dev/spidev0.0", O_RDWR);

	unsigned int speed = 1000000;
	ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	int last_counter_received = 0;

	/**********************************************************
An endless loop that repeatedly sends the demonstration
commands to the Arduino and displays the results
	 ***********************************************************/
	long missed = 0;
	long passed = 0;
	struct timeval start;
	struct timeval now;
	struct timeval last;
	struct timeval last_printtime;
	gettimeofday(&start,NULL);
	gettimeofday(&now,NULL);
	gettimeofday(&last,NULL);
	gettimeofday(&last_printtime,NULL);
	while (1)
	{
		if(query_message == 1)
		{
			unsigned char query = query_type;
			unsigned char inputbuffer[12];
			int success;
			int length;
			int passed_checksum_calc = sendQuery(query,inputbuffer);
			if(passed_checksum_calc > 0)
			{
				passed_checksum++;
			}
			else if(passed_checksum_calc == 0)
			{
				failed_checksum++;
			}
			else
			{
				return 0;
			}

			unsigned char v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12;
			uint16_t a1,a2,a3,a4,a5,a6;
			switch(query)
			{
			case SPIMessageHandler::SPI_Diagnostic_ID:
				success = spimessagehandler->decode_DiagnosticSPI(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6);
				if(success == 1)
				{
					printf("%d Diagnostic: %d %d %d %d %d %d\n",
							passed_checksum_calc,v1,v2,v3,v4,v5,v6);
				}
				break;
			case SPIMessageHandler::SPI_TestMessageCounter_ID:
				success = spimessagehandler->decode_TestMessageCounterSPI(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8,&v9,&v10,&v11,&v12);
				if(success == 1)
				{
					printf("%d %d %d %d %d %d %d %d %d %d %d %d\n",
							v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12);
					if(first_message_received == 0)
					{
						first_message_received = 1;

					}
					else
					{
						if((int)v1 < 255)
						{
							if(((int)v1-last_counter_received) != 6)
							{
								missed++;
							}
							else
							{
								passed++;
							}
						}

					}
					last_counter_received = (int)v1;
				}
				else
				{
					printf("Unable to Decode\n");
				}
				printf("Missed: %ld @ %f Passed: %ld @ %f Succeed Ratio: %f%%\n",
						missed,missed/(dt(start,now)),
						passed,passed/(dt(start,now)),
						100.0*(double)passed/((double)(passed)+(double)(missed)));
				break;
			case SPIMessageHandler::SPI_Get_ANA_Port1_ID:
				success = spimessagehandler->decode_Get_ANA_Port1SPI(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == 1)
				{
					printf("ANA Port1 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d\n",
							a1,a2,a3,a4,a5,a6);
				}
				break;
			case SPIMessageHandler::SPI_Get_DIO_Port1_ID:
				success = spimessagehandler->decode_Get_DIO_Port1SPI(inputbuffer,&length,&a1,&a2);
				if(success == 1)
				{
					printf("%d DIO Port1 0:%d 1:%d\n",
							passed_checksum_calc,a1-BYTE2_OFFSET,a2-BYTE2_OFFSET);
				}
				break;
			default:
				break;
			}
			gettimeofday(&now,NULL);
			gettimeofday(&last,NULL);
			if(dt(last_printtime,now) > 1.0)
			{
				printf("Passed Checksum: %ld @ %f Failed Checksum: %ld @ %f Succeed Ratio: %f%%\n",
						passed_checksum,passed_checksum/(dt(start,now)),
						failed_checksum,failed_checksum/(dt(start,now)),
						100.0*(double)passed_checksum/((double)passed_checksum+(double)failed_checksum));
				gettimeofday(&last_printtime,NULL);
			}
			usleep(loop_delay);

		}
		else if(command_message == 1)
		{
			unsigned char command = command_type;
			unsigned char outputbuffer[12];
			int success;
			int length;
			switch(command)
			{
			case SPIMessageHandler::SPI_LEDStripControl_ID:
				success = spimessagehandler->encode_LEDStripControlSPI(outputbuffer,&length,param1,3,4);
				if(success == 0)
				{
					printf("[ERROR]: Unable to encode SPI Message: %d\n",command);
				}
				break;
			case SPIMessageHandler::SPI_Arm_Status_ID:
				success = spimessagehandler->encode_Arm_StatusSPI(outputbuffer,&length,param1);
				if(success == 0)
				{
					printf("[ERROR]: Unable to encode SPI Message: %d\n",command);
				}
				break;
			case SPIMessageHandler::SPI_Command_ID:
				success = spimessagehandler->encode_CommandSPI(outputbuffer,&length,param1,param2,param3,param4);
				if(success == 0)
				{
					printf("[ERROR]: Unable to encode SPI Message: %d\n",command);
				}
				break;
			default:
				break;
			}
			
			sendCommand(command,outputbuffer);
			usleep(loop_delay);
		}
	}


}
double dt(struct timeval a, struct timeval b)
{
	double t1 = (double)(a.tv_sec) + (double)(a.tv_usec)/1000000.0;
	double t2 = (double)(b.tv_sec) + (double)(b.tv_usec)/1000000.0;
	return t2-t1;
}

int spiTxRx(unsigned char txDat)
{

	unsigned char rxDat;

	struct spi_ioc_transfer spi;

	memset (&spi, 0, sizeof (spi));

	spi.tx_buf        = (unsigned long)&txDat;
	spi.rx_buf        = (unsigned long)&rxDat;
	spi.len           = 1;

	ioctl (fd, SPI_IOC_MESSAGE(1), &spi);
	return rxDat;
}
int sendCommand(unsigned char command,unsigned char* outputbuffer)
{
	unsigned char resultByte;
	bool ack;
	int wait_time_us = 1;
	int counter = 0;
	do
	{
		ack = false;

		spiTxRx(0xAB);
		usleep (wait_time_us);


		resultByte = spiTxRx(command);
		if (resultByte == 'a')
		{
			ack = true;
		}
		else { counter++; }
		if(counter > 10000)
		{
			printf("No Comm with device after %d tries. Exiting.\n",counter);
			return -1;
		}
		usleep (wait_time_us);
	}

	while (ack == false);
	usleep(wait_time_us);
	unsigned char v;
	unsigned char running_checksum = 0;
	for(int i = 0; i < 12; i++)
	{
		v = spiTxRx(outputbuffer[i]);
		running_checksum ^= v;
		outputbuffer[i] = v;
		//*p_outbuffer++ = v;
		usleep(wait_time_us);

	}

	resultByte = spiTxRx(running_checksum);
	usleep(wait_time_us);
	return 1;
}
int sendQuery(unsigned char query, unsigned char * inputbuffer)
{
	unsigned char resultByte;
	bool ack;
	int wait_time_us = 1;
	int counter = 0;
	do
	{
		ack = false;

		spiTxRx(0xAB);
		usleep (wait_time_us);


		resultByte = spiTxRx(query);
		if (resultByte == 'a')
		{
			ack = true;
		}
		else { counter++; }
		if(counter > 10000)
		{
			printf("No Comm with device after %d tries. Exiting.\n",counter);
			return -1;
		}
		usleep (wait_time_us);
	}

	while (ack == false);
	usleep(wait_time_us);
	resultByte = spiTxRx(0);
	usleep(wait_time_us);
	unsigned char v;
	unsigned char running_checksum = 0;
	for(int i = 0; i < 12; i++)
	{
		v = spiTxRx(0);
		running_checksum ^= v;
		inputbuffer[i] = v;
		usleep(wait_time_us);

	}
	resultByte = spiTxRx(0);
	usleep(wait_time_us);
	if(resultByte == running_checksum) { return 1; }
	else { return 0; }
}
