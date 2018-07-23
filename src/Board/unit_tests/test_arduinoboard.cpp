#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include "spimessage.h"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
using namespace std;

static void show_usage(std::string name)
{
	std::cerr << "Usage: Test SPI Comm between Raspberry Pi and Arduino. Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-d,--delay Delay in MicroSeconds between sending each message.  Default is 100000.\n"
			<< "\t-q,--query Query Message.  Supported messages are:\n"

			<< "\t\t [0] TestMessageCounter (0xAB14)\n"
			<< "\t\t [1] Get_DIO_Port1 (0xAB19)\n"
			<< "\t\t [2] Get_ANA_Port1 (0xAB20)\n"
			<< "\t-t,--test Send Test Message.\n"
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
int sendMessage();
void transfer(int fd);
double dt(struct timeval a, struct timeval b);

/**********************************************************
Main
 ***********************************************************/

int main(int argc, char* argv[])
{
	passed_checksum = 0;
	failed_checksum = 0;
	int query_message = 0;
	unsigned char query_type;
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
		else if ((arg == "-q") || (arg == "--query"))
		{
			query_message = 1;
			if (i + 1 < argc)
			{
				int v = atoi(argv[i+1]);
				switch(v)
				{
				case 0:
					query_type = SPIMessageHandler::SPI_TestMessageCounter_ID;
					break;
				case 1:
					query_type = SPIMessageHandler::SPI_Get_DIO_Port1_ID;
					break;
				case 2:
					query_type = SPIMessageHandler::SPI_Get_ANA_Port1_ID;
					break;
				default:
					printf("Unsupported Query Message.  Exiting.\n");
					return 0;
				}
				i++;
			}
		}
		else if((arg == "-t" ) || (arg == "--test"))
		{

			//printf("Sending: %s\n",argv[i+1]);
			int send_count = sendMessage();//argv[i+1]);
			printf("Sent: %d Bytes\n",send_count);
			return 0;
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
	char command[2];
	while (1)
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
			printf("Missed: %d @ %f Passed: %d @ %f Succeed Ratio: %f%\n",
					missed,missed/(dt(start,now)),
					passed,passed/(dt(start,now)),
					100.0*(double)passed/((double)passed+(double)missed));
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
			success = spimessagehandler->decode_Get_DIO_Port1SPI(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
			if(success == 1)
			{
				printf("DIO Port1 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6: %d 7: %d\n",
						v1,v2,v3,v4,v5,v6,v7,v8);
			}
			break;
		default:
			break;
		}
		gettimeofday(&now,NULL);
		gettimeofday(&last,NULL);
		if(dt(last_printtime,now) > 1.0)
		{
			printf("Passed Checksum: %d @ %f Failed Checksum: %d @ %f Succeed Ratio: %f%\n",
					passed_checksum,passed_checksum/(dt(start,now)),
					failed_checksum,failed_checksum/(dt(start,now)),
					100.0*(double)passed_checksum/((double)passed_checksum+(double)failed_checksum));
			gettimeofday(&last_printtime,NULL);
		}
		usleep(loop_delay);

	}

}
double dt(struct timeval a, struct timeval b)
{
	double t1 = (double)(a.tv_sec) + (double)(a.tv_usec)/1000000.0;
	double t2 = (double)(b.tv_sec) + (double)(b.tv_usec)/1000000.0;
	return t2-t1;
}
int sendMessage()
{
	uint8_t bits = 8;
	uint32_t speed = 500000;
	uint16_t delay;
	int fd = open("/dev/spidev0.0", O_RDWR);
	if (fd < 0)
	{
		printf("can't open device\n");
	}
	uint8_t mode;
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	printf("mode: %d\n",mode);
	/*if (ret == -1)
	{
		printf("can't set spi mode\n");
	}
	*/
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		printf("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		printf("can't get bits per word");
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		printf("can't set max speed hz");
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		printf("can't get max speed hz");
	}
	char message[] = "Hello,World!";
	transfer(fd);
	/*for(int i = 0; i < (unsigned)strlen(message);i++)
	{
		spiTxRx(message[i]);
	}
	*/
	close(fd);
	return 0;
}
void transfer(int fd)
{
	int ret;
	uint8_t tx[] = {
        0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0x20,
        0x57, 0x4F, 0x52, 0x4C, 0x44,
        0x0A
	};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer spi;

		memset (&spi, 0, sizeof (spi));
		spi.tx_buf = (unsigned long)&tx;
		spi.rx_buf = (unsigned long)&rx;
		spi.len = ARRAY_SIZE(tx);
		spi.delay_usecs = 0;
		spi.speed_hz = 500000;
		spi.bits_per_word = 8;
	/*
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = 0,
		.speed_hz = 500000,
		.bits_per_word = 8,
	};
	*/

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tx);
	if (ret < 1)
		printf("can't send spi message");


	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%c ", rx[ret]);
	}
	puts("");

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
		//*p_outbuffer++ = v;
		usleep(wait_time_us);

	}
	resultByte = spiTxRx(0);
	usleep(wait_time_us);
	if(resultByte == running_checksum) { return 1; }
	else { return 0; }
}
