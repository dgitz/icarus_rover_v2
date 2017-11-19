#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string.h>
#include <sys/time.h>
//#include "../Driver/ServoHatDriver.h"

using namespace std;
double measure_timediff(timeval a, timeval b);
static void show_usage(std::string name)
{
    std::cerr << "Usage: Test ServoHat. Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-m,--mode Mode\traw,stat.\n"
              << "\t-d,--device SerialDevice.\n"
              << "\t-b,--baudrate BaudRate.\n"
              << std::endl;
}

int main(int argc, char* argv[])
{
    std::string mode = "";
    std::string device = "";
    std::string baudrate= "";
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
        else if ((arg == "-m") || (arg == "--mode"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                mode = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-d") || (arg == "--device"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                device = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-b") || (arg == "--baudrate"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                baudrate = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
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
    printf("Testing Serial Comm with: device=%s baudrate=%s mode=%s\n",device.c_str(),baudrate.c_str(),mode.c_str());
    int dev_fd = open(device.c_str(),O_RDWR | O_NOCTTY);
	struct termios tty;
	memset(&tty,0,sizeof tty);
	if(tcgetattr(dev_fd,&tty) != 0 )
	{
		std::cout << "Error: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}
    if(baudrate == "115200")
    {
        cfsetospeed(&tty,(speed_t)B115200);
        cfsetispeed(&tty,(speed_t)B115200);
    }
    else
    {
        printf("Baudrate: %s Not Supported.\n",baudrate.c_str());
    }
	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);
	tcflush( dev_fd, TCIFLUSH );
	if ( tcsetattr ( dev_fd, TCSANOW, &tty ) != 0) {
	   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	}
    int packet_counter = 0;
    long rx_counter = 0;
    timeval now,last;
    gettimeofday(&now,NULL);
    last = now;
    double run_time = 0.0;
    double medium_timer = 0.0;
    while(1)
    {
        gettimeofday(&now,NULL);
        double dt = measure_timediff(now,last);  
        run_time += dt;
        medium_timer += dt;
        int n = 0;
		char response[1024];
		int spot = 0;
		char buf = '\0';
		memset(response, '\0', sizeof response);
		do
		{
			n = read( dev_fd, &buf, 1 );
            rx_counter += n;
			sprintf( &response[spot], "%c", buf );
			spot += n;
		} while( buf != '\r' && n > 0);
        if (n < 0)
        {
            std::cout << "Error reading: " << strerror(errno) << std::endl;
        }
        else if (n == 0)
        {
            std::cout << "Read nothing!" << std::endl;
        }
        else
        {
            packet_counter++;
            if(mode=="raw")
            {
                printf("%s",response);
            }
            
        }
        
        if(medium_timer > 0.25)
        {
            medium_timer = 0.0;
            if(mode=="stat")
            {
                printf("Rx: %d @ Rate: %f bps: %f Bpp: %f\n",packet_counter,(double)(packet_counter)/run_time,8.0*(double)(rx_counter)/run_time,(double)(rx_counter)/(double)(packet_counter));                
            }
        }
        last = now;
    }
    close(dev_fd);
	return 0;
}
double measure_timediff(timeval b, timeval a)
{
    double t1 = (double)(a.tv_sec) + (double)(a.tv_usec)/1000000.0;
    double t2 = (double)(b.tv_sec) + (double)(b.tv_usec)/1000000.0;
    return t2-t1;
}