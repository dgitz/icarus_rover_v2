#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../../include/i2cmessage.h"
#include "../Driver/GPIOHatDriver.h"

using namespace std;

static void show_usage(std::string name)
{
	std::cerr << "Usage: Test GPIOHat. Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-a,--address Address\tThe I2C Address of the Hat (Base 10).  Default=20.\n"
			<< "\t-d,--delay Delay in MicroSeconds between sending each message.  Default is 100000.\n"
			<< "\t-q,--query\tQuery Device for Data.  Options:\n"
			<< "\t\t [0] Get_Diagnostic (0xAB12)\n"
			<< "\t\t [1] TestMessageCounter (0xAB14)\n"
			<< "\t\t [2] Get_DIO_Port1 (0xAB19)\n"
			<< "\t\t [3] Get_IMU Accel (0xAB27)\n"
			<< "\t\t [4] Get_IMU Gyro (0xAB28)\n"
			<< "\t\t [5] Get_IMU Mag (0xAB29)\n"
			<< std::endl;
}
/**********************************************************
Housekeeping variables
 ***********************************************************/
int results;
int first_message_received;
long passed_checksum;
long failed_checksum;
I2CMessageHandler *i2cmessagehandler;

/**********************************************************
Declare Functions
 ***********************************************************/
double dt(struct timeval a, struct timeval b);

int main(int argc, char* argv[])
{
	int hat_address = 20;
	int option = -1;
	long loop_delay = 100000;
	bool query_message = false;
	unsigned char query_type;
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
		else if ((arg == "-a") || (arg == "--address"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				hat_address = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--address option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-q") || (arg == "--query"))
		{
			if (i + 1 < argc)
			{
				if (i + 1 < argc)
				{
					int v = atoi(argv[i+1]);
					switch(v)
					{
					case 0:
						query_type = I2CMessageHandler::I2C_Diagnostic_ID;
						break;
					case 1:
						query_type = I2CMessageHandler::I2C_TestMessageCounter_ID;
						break;
					case 2:
						query_type = I2CMessageHandler::I2C_Get_DIO_Port1_ID;
						break;
					case 3:
						query_type = I2CMessageHandler::I2C_Get_IMUAcc_ID;
						break;
					case 4:
						query_type = I2CMessageHandler::I2C_Get_IMUGyro_ID;
						break;
					case 5:
						query_type = I2CMessageHandler::I2C_Get_IMUMag_ID;
						break;
					default:
						printf("Unsupported Query Message.  Exiting.\n");
						return 0;
					}
					query_message = true;
					i++;
				}
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--address option requires one argument." << std::endl;
				return 1;
			}
		}

	}
	GPIOHatDriver gpiohat;
	printf("Using address: %d\n",hat_address);
	int status = gpiohat.init(hat_address);
	int last_counter_received = 0;
	int counter = 0;
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
	char receive[16];
	while (1)
	{
		if(query_message)
		{
			unsigned char query = query_type;
			unsigned char inputbuffer[12];
			int success;
			int length;
			if(gpiohat.get_lock() == false)
			{
				gpiohat.lockdevice();

				int passed_checksum_calc = gpiohat.sendQuery(query,inputbuffer);
				if(passed_checksum_calc > 0)
				{
					passed_checksum++;
				}
				else if(passed_checksum_calc == 0)
				{
					failed_checksum++;
					continue;
				}
				else
				{
					return 0;
				}

				unsigned char v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12;
				uint16_t a1,a2,a3,a4,a5,a6;
				switch(query)
				{
				case I2CMessageHandler::I2C_Diagnostic_ID:
					success = i2cmessagehandler->decode_DiagnosticI2C(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6);
					if(success == 1)
					{
						printf("Diagnostic: %d %d %d %d %d %d\n",
								passed_checksum_calc,v1,v2,v3,v4,v5,v6);
					}
					break;
				case I2CMessageHandler::I2C_Get_DIO_Port1_ID:
					success = i2cmessagehandler->decode_Get_DIO_Port1I2C(inputbuffer,&length,&a1,&a2,&a3,&a4);
					if(success == 1)
					{
						printf("%d DIO Port1 0:%d 1:%d 2:%d 3:%d\n",
								passed_checksum_calc,a1,a2,a3,a4);
					}
					break;
				case I2CMessageHandler::I2C_Get_IMUAcc_ID:
					success = i2cmessagehandler->decode_Get_IMUAccI2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
					if(success == 1)
					{
						double x1,y1,z1,x2,y2,z2;
						x1 = (double)(a1-32768)/100.0;
						y1 = (double)(a2-32768)/100.0;
						z1 = (double)(a3-32768)/100.0;
						x2 = (double)(a4-32768)/100.0;
						y2 = (double)(a5-32768)/100.0;
						z2 = (double)(a6-32768)/100.0;
						printf("%d ACC 0:%f 1:%f 2:%f 3:%f 4: %f 5: %f\n",
								passed_checksum_calc,x1,y1,z1,x2,y2,z2);
					}
					break;
				case I2CMessageHandler::I2C_Get_IMUGyro_ID:
					success = i2cmessagehandler->decode_Get_IMUGyroI2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
					if(success == 1)
					{
						double x1,y1,z1,x2,y2,z2;
						x1 = (double)(a1-32768)/1000.0;
						y1 = (double)(a2-32768)/1000.0;
						z1 = (double)(a3-32768)/1000.0;
						x2 = (double)(a4-32768)/1000.0;
						y2 = (double)(a5-32768)/1000.0;
						z2 = (double)(a6-32768)/1000.0;
						printf("%d GYRO 0:%f 1:%f 2:%f 3:%f 4: %f 5: %f\n",
								passed_checksum_calc,x1,y1,z1,x2,y2,z2);
					}
					break;
				case I2CMessageHandler::I2C_Get_IMUMag_ID:
					success = i2cmessagehandler->decode_Get_IMUMagI2C(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
					if(success == 1)
					{
						double x1,y1,z1,x2,y2,z2;
						x1 = (double)(a1-32768)/1000.0;
						y1 = (double)(a2-32768)/1000.0;
						z1 = (double)(a3-32768)/1000.0;
						x2 = (double)(a4-32768)/1000.0;
						y2 = (double)(a5-32768)/1000.0;
						z2 = (double)(a6-32768)/1000.0;
						printf("%d MAG 0:%f 1:%f 2:%f 3:%f 4: %f 5: %f\n",
								passed_checksum_calc,x1,y1,z1,x2,y2,z2);
					}
					break;
				default:
					break;
				case I2CMessageHandler::I2C_TestMessageCounter_ID:
					success = i2cmessagehandler->decode_TestMessageCounterI2C(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8,&v9,&v10,&v11,&v12);
					if(success == 1)
					{
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

				}
			}
		}
		else
		{
			printf("Nothing To Do.  Exiting.\n");
			return 0;
		}
		gettimeofday(&now,NULL);
		gettimeofday(&last,NULL);
		if(dt(last_printtime,now) > 1.0)
		{
			if(query_type == I2CMessageHandler::I2C_TestMessageCounter_ID)
			{
				printf("Missed: %d @ %f Passed: %d @ %f Succeed Ratio: %f%\n",
						missed,missed/(dt(start,now)),
						passed,passed/(dt(start,now)),
						100.0*(double)passed/((double)passed+(double)missed));
			}
			printf("Passed Checksum: %d @ %f Failed Checksum: %d @ %f Succeed Ratio: %f%\n",
					passed_checksum,passed_checksum/(dt(start,now)),
					failed_checksum,failed_checksum/(dt(start,now)),
					100.0*(double)passed_checksum/((double)passed_checksum+(double)failed_checksum));
			gettimeofday(&last_printtime,NULL);
		}

	}
	return 0;
}
double dt(struct timeval a, struct timeval b)
{
	double t1 = (double)(a.tv_sec) + (double)(a.tv_usec)/1000000.0;
	double t2 = (double)(b.tv_sec) + (double)(b.tv_usec)/1000000.0;
	return t2-t1;
}
