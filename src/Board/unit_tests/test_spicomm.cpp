/**********************************************************
 SPI_Raspi_Arduino
   Configures an Raspberry Pi as an SPI master and
   demonstrates a basic bidirectional communication scheme
   with an Arduino slave.  The Raspberry Pi transmits
   commands to perform addition and subtraction on a pair
   of integers and the Ardunio returns the result

Compile String:
g++ -o SPI_Raspi_Arduino SPI_Raspi_Arduino.cpp
***********************************************************/

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
using namespace std;

static void show_usage(std::string name)
{
    std::cerr << "Usage: Test SPI Comm between Raspberry Pi and Arduino. Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-d,--delay Delay in MicroSeconds between sending each message.  Default is 100000.\n"
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

/**********************************************************
Declare Functions
***********************************************************/

int spiTxRx(unsigned char txDat);
int sendCommand(char command);
double dt(struct timeval a, struct timeval b);

/**********************************************************
Main
***********************************************************/

int main(int argc, char* argv[])
{

/**********************************************************
Setup SPI
Open file spidev0.0 (chip enable 0) for read/write access
with the file descriptor "fd"
Configure transfer speed (1MkHz)
***********************************************************/
	passed_checksum = 0;
	failed_checksum = 0;
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
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--delay option requires one argument." << std::endl;
                return 1;
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
	char command[2];
   while (1)
   {
      results = sendCommand(0x14);
	  if(results < 0)
	  {
		return 0;
	  }
	  //printf("Got: %d expected: %d\n",(int)results,last_counter_received+6);
	  if(first_message_received == 0)
	  {
		first_message_received = 1;
		
	  }
	  else
	  {
		if((int)results < 255)
		{
			if(((int)results-last_counter_received) != 6)
			{
				missed++;
			}
			else
			{
				passed++;
			}
		}
		
	  }
	  last_counter_received = (int)results;
		//if((int)(results) != 1165) { error++;}
      //cout << "Addition results:" << endl;
     //cout << "510 + 655 = " <<  (int)(results) << endl;

	//printf("error: %d\n",error);
	gettimeofday(&now,NULL);
	//printf("loop dt: %f\n",1.0/(dt(last,now)));
	gettimeofday(&last,NULL);
	//printf("error count: %d rate: %f elap time: %f\n",error,error/(dt(start,now)),dt(start,now));
	if(dt(last_printtime,now) > 1.0)
	{
		printf("Passed Checksum: %d @ %f Failed Checksum: %d @ %f Missed: %d @ %f Passed: %d @ %f Succeed Ratio: %f%\n",
			passed_checksum,passed_checksum/(dt(start,now)),
			failed_checksum,failed_checksum/(dt(start,now)),
			missed,missed/(dt(start,now)),
			passed,passed/(dt(start,now)),
			100.0*(double)passed/((double)passed+(double)missed));
		gettimeofday(&last_printtime,NULL);	
		}
    usleep(loop_delay);

     }

}

/**********************************************************
spiTxRx
 Transmits one byte via the SPI device, and returns one byte
 as the result.

 Establishes a data structure, spi_ioc_transfer as defined
 by spidev.h and loads the various members to pass the data
 and configuration parameters to the SPI device via IOCTL

 Local variables txDat and rxDat are defined and passed by
 reference.  
***********************************************************/
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


/**********************************************************
sendCommand
 Demonstration of a protocol that uses the spiTxRx function
 to send a formatted command sequence/packet to the Arduino
 one byte at and capture the results
***********************************************************/


int sendCommand(char command)
{

unsigned char resultByte;
bool ack;

/**********************************************************
Unions allow variables to occupy the same memory space
a convenient way to move back and forth between 8-bit and
16-bit values etc.

Here three unions are declared: two for parameters to be 
passed in commands to the Arduino and one to receive
the results
***********************************************************/


/**********************************************************
An initial handshake sequence sends a one byte start code
('c') and loops endlessly until it receives the one byte 
acknowledgment code ('a') and sets the ack flag to true.
(Note that the loop also sends the command byte while 
still in handshake sequence to avoid wasting a transmit
cycle.)
***********************************************************/
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
/**********************************************************
Send the parameters one byte at a time.
***********************************************************/

/**********************************************************
Push two more zeros through so the Arduino can return the
results
***********************************************************/

  usleep(wait_time_us);
  resultByte = spiTxRx(0);
  usleep(wait_time_us);
  unsigned char v;
  unsigned char running_checksum = 0;
  for(int i = 0; i < 12; i++)
  {
	  v = spiTxRx(0);
	  running_checksum ^= v;
	  usleep(wait_time_us);
	  //printf("i: %d v: %d \n",i,v);
	}
	//printf("\n");
  resultByte = spiTxRx(0);
  usleep(wait_time_us);
  if(resultByte == running_checksum) { passed_checksum++; }
  else { failed_checksum++; }
  return v;

}
