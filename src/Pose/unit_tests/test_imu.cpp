#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../Driver/IMUDriver.h"
#define GREEN_FOREGROUND "\033[1;32m"
#define YELLOW_FOREGROUND "\033[1;33m"
#define RED_FOREGROUND "\033[1;31m"
#define END_COLOR "\033[0m"
using namespace std;

static void show_usage(std::string name)
{
	std::cerr << "Usage: Test IMU. Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-d,--delay Delay (uS)\t\t Delay in micro Seconds.  Default is 100000.\n"
			<< "\t-m,--mode\tMode: monitor. Default=monitor.\n"
            << "\t-r,--rate\tRate: Update Rate of IMU.  Default=50.\n"
            << "\t-v,--verbose\tVerbose.\n"
			<< "\t-p,--port\tSerial Port. Default=/dev/ttyAMA0.\n"
			<< "\t-b,--baudrate\tBaud Rate. Default=115200.\n"
			<< std::endl;
}
void print_imudata(IMUDriver driver,IMUDriver::RawIMU imu_data);
int main(int argc, char* argv[])
{
	std::string mode = "monitor";
	int delay = 100000;
    double rate = 50.0;
	std::string port = "/dev/ttyAMA0";
	std::string baudrate = "115200";
    bool verbose = false;
	struct timeval start_time,now,last;
	gettimeofday(&now,NULL);
	gettimeofday(&last,NULL);
	gettimeofday(&start_time,NULL);
    IMUDriver::RawIMU imu_data;
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
				std::cerr << "--mode option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-d") || (arg == "--delay"))
		{
			if (i + 1 < argc)
			{
				delay = atoi(argv[i+1]);
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--Pin Mode option requires one argument." << std::endl;
				return 1;
			}
		}
        else if ((arg == "-v") || (arg == "--verbose"))
		{
			i++;
            verbose = true;
		}
		else
		{
		}
	}
	IMUDriver imu;
	int status = imu.init("serial",port,baudrate);
	if(status < 0)
    {
        printf("[IMU]: Unable to Initialize. Exiting.\n");
        return 0;
    }
	while(1)
	{
		if(mode == "monitor")
		{
			gettimeofday(&now,NULL);
			imu_data = imu.update();
            if(verbose == true)
            {
                printf("[IMU] Raw:%s\n",imu.get_rawdata().c_str());
            }
            print_imudata(imu,imu_data);



		}
		usleep(delay);
	}
	imu.finish();
	return 0;
}
void print_imudata(IMUDriver driver,IMUDriver::RawIMU imu_data)
{
    std::string start_color = "";
    std::string end_color = "";
    switch(imu_data.signal_state)
    {
        case SIGNALSTATE_UNDEFINED:
            start_color = RED_FOREGROUND;
            end_color = END_COLOR;
            break;
        case SIGNALSTATE_INVALID:
            start_color = RED_FOREGROUND;
            end_color = END_COLOR;
            break;
        case SIGNALSTATE_INITIALIZING:
            start_color = YELLOW_FOREGROUND;
            end_color = END_COLOR;
            break;
        case SIGNALSTATE_UPDATED:
            start_color = GREEN_FOREGROUND;
            end_color = END_COLOR;
            break;
        case SIGNALSTATE_HOLD:
            start_color = YELLOW_FOREGROUND;
            end_color = END_COLOR;
            break;
        case SIGNALSTATE_CALIBRATING:
            start_color = YELLOW_FOREGROUND;
            end_color = END_COLOR;
            break;
    }

    printf("%s[IMU] T=%4.2f U=%ld R=%4.2f State: %s Acc: X=%4.2f Y=%4.2f Z=%4.2f Gyro: X=%4.2f Y=%4.2f Z=%4.2f Mag: X=%4.2f Y=%4.2f Z=%4.2f\n%s",
        start_color.c_str(),
        imu_data.tov,
        imu_data.update_count,
        imu_data.update_rate,
        driver.map_signalstate_tostring(imu_data.signal_state).c_str(),
        imu_data.acc_x,
        imu_data.acc_y,
        imu_data.acc_z,
        imu_data.gyro_x,
        imu_data.gyro_y,
        imu_data.gyro_z,
        imu_data.mag_x,
        imu_data.mag_y,
        imu_data.mag_z,
        end_color.c_str()
    );
        
}
