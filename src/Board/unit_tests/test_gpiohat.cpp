#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../../include/i2cmessage.h"
#include "../../../include/Supported_PN.h"
#include "../Driver/GPIOHatDriver.h"

using namespace std;

static void show_usage()
{
	std::cerr 	<< "This program is used to test the operation of a directly connected GPIOHat.\n"
				<< "Currently supported Part Numbers:\n"
				<< "\tPN: " << PN_100007 << "\n"
			  	<< "Usage: Test GPIOHat via I2C. Options:\n"
			 	<< "\t-h,--help\t\tShow this help message\n"
			  	<< "\t-a,--address Address\tThe I2C Address of the Hat (Base 10).  Default=20.\n"
			  	<< "\t-d,--delay\t\tDelay in MicroSeconds between sending each message.  Default is 100000.\n"
			  	<< "\t-q,--query\t\tQuery Device for Data.  Options:\n"
			  	<< "\t\t [0] Get_Diagnostic (0xAB12)\n"
			  	<< "\t\t [1] TestMessageCounter (0xAB14)\n"
			  	<< "\t\t [2] Get_DIO_Port1 (0xAB19)\n"
			  	<< "\t\t [2] Get_ANA_Port1 (0xAB20)\n"
			  	<< "\t\t [2] Get_ANA_Port2 (0xAB21)\n"
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

int main(int argc, char *argv[])
{
	int hat_address = 20;
	int option = -1;
	long loop_delay = 100000;
	bool query_message = false;
	unsigned char query_type;
	if (argc < 2)
	{
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
				loop_delay = atoi(argv[i + 1]); // Increment 'i' so we don't get the argument as the next argv[i
				i++;
			}
		}
		else if ((arg == "-a") || (arg == "--address"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				hat_address = atoi(argv[i + 1]); // Increment 'i' so we don't get the argument as the next argv[i
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
					int v = atoi(argv[i + 1]);
					switch (v)
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
						query_type = I2CMessageHandler::I2C_Get_ANA_Port1_ID;
						break;
					case 4:
						query_type = I2CMessageHandler::I2C_Get_ANA_Port2_ID;
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
	printf("Using address: %d\n", hat_address);
	int status = gpiohat.init(hat_address);
	int last_counter_received = 0;
	int counter = 0;
	long missed = 0;
	long passed = 0;
	struct timeval start;
	struct timeval now;
	struct timeval last;
	struct timeval last_printtime;
	gettimeofday(&start, NULL);
	gettimeofday(&now, NULL);
	gettimeofday(&last, NULL);
	gettimeofday(&last_printtime, NULL);
	char receive[16];
	while (1)
	{
		if (query_message)
		{
			unsigned char query = query_type;
			unsigned char inputbuffer[12];
			int success;
			int length;

			int passed_checksum_calc = gpiohat.sendQuery(query, inputbuffer);
			if (passed_checksum_calc > 0)
			{
				passed_checksum++;
			}
			else if (passed_checksum_calc == 0)
			{
				failed_checksum++;
				continue;
			}
			else
			{
				return 0;
			}

			unsigned char v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
			uint16_t a1, a2, a3, a4, a5, a6;
			switch (query)
			{
			case I2CMessageHandler::I2C_Diagnostic_ID:
				success = i2cmessagehandler->decode_DiagnosticI2C(inputbuffer, &length, &v1, &v2, &v3, &v4, &v5, &v6);
				if (success == 1)
				{
					printf("Diagnostic: %d %d %d %d %d %d\n",
						   passed_checksum_calc, v1, v2, v3, v4, v5, v6);
				}
				break;
			case I2CMessageHandler::I2C_Get_DIO_Port1_ID:
				success = i2cmessagehandler->decode_Get_DIO_Port1I2C(inputbuffer, &length, &a1, &a2, &a3, &a4);
				if (success == 1)
				{
					printf("%d DIO Port1 0:%d 1:%d 2:%d 3:%d\n",
						   passed_checksum_calc, a1, a2, a3, a4);
				}
				break;
			case I2CMessageHandler::I2C_Get_ANA_Port1_ID:
				success = i2cmessagehandler->decode_Get_ANA_Port1I2C(inputbuffer, &length, &a1, &a2, &a3, &a4, &a5, &a6);
				if (success == 1)
				{
					printf("%d ANA Port1 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d\n",
						   passed_checksum_calc, a1, a2, a3, a4, a5, a6);
				}
				break;
			case I2CMessageHandler::I2C_Get_ANA_Port2_ID:
				success = i2cmessagehandler->decode_Get_ANA_Port2I2C(inputbuffer, &length, &a1, &a2, &a3, &a4, &a5, &a6);
				if (success == 1)
				{
					printf("%d ANA Port2 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d\n",
						   passed_checksum_calc, a1, a2, a3, a4, a5, a6);
				}
				break;
			case I2CMessageHandler::I2C_TestMessageCounter_ID:
				success = i2cmessagehandler->decode_TestMessageCounterI2C(inputbuffer, &length, &v1, &v2, &v3, &v4, &v5, &v6, &v7, &v8, &v9, &v10, &v11, &v12);
				if (success == 1)
				{
					if (first_message_received == 0)
					{
						first_message_received = 1;
					}
					else
					{
						if ((int)v1 < 255)
						{
							if (((int)v1 - last_counter_received) != 6)
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
			default:
				break;
			}
		}
		else
		{
			printf("Nothing To Do.  Exiting.\n");
			return 0;
		}
		gettimeofday(&now, NULL);
		gettimeofday(&last, NULL);
		if (dt(last_printtime, now) > 1.0)
		{
			if (query_type == I2CMessageHandler::I2C_TestMessageCounter_ID)
			{
				printf("Missed: %d @ %f Passed: %d @ %f Succeed Ratio: %f%\n",
					   missed, missed / (dt(start, now)),
					   passed, passed / (dt(start, now)),
					   100.0 * (double)passed / ((double)passed + (double)missed));
			}
			printf("Passed Checksum: %d @ %f Failed Checksum: %d @ %f Succeed Ratio: %f%\n",
				   passed_checksum, passed_checksum / (dt(start, now)),
				   failed_checksum, failed_checksum / (dt(start, now)),
				   100.0 * (double)passed_checksum / ((double)passed_checksum + (double)failed_checksum));
			gettimeofday(&last_printtime, NULL);
		}
	}
	return 0;
}
double dt(struct timeval a, struct timeval b)
{
	double t1 = (double)(a.tv_sec) + (double)(a.tv_usec) / 1000000.0;
	double t2 = (double)(b.tv_sec) + (double)(b.tv_usec) / 1000000.0;
	return t2 - t1;
}
