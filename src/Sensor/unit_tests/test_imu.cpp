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

static void show_usage()
{
	std::cerr << "This program is used to test the operation of a directly connected IMU.\n"
			<< "Currently supported Part Numbers:\n"
			<< "\tPN: " << PN_110012 << "\n"
			<< "\tPN: " << PN_110015 << "\n"
			<<  "Usage: Tests IMU via UART Connection. Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-d,--delay Delay (uS)\tDelay in micro Seconds.  Default is 100000.\n"
			<< "\t-m,--mode\t\tMode: monitor,query. Default=monitor.\n"
			<< "\t-r,--report\t\tReport: stat,acc,gyro,mag,all. Default=all.\n"
			<< "\t-v,--verbose\t\tVerbosity: Default=0.\n"
			<< "\t-pn,--partno\t\tPart Number: 110012,110015.\n"
			<< "\t-p,--port\t\tSerial Port. Default:\n\t\tPN=110012:/dev/ttyAMA0\n\t\tPN=110015:/dev/ttyACM0\n"
			<< std::endl;
}
void print_imudata(std::string report,IMUDriver driver,IMUDriver::RawIMU imu_data);
int main(int argc, char* argv[])
{
	std::string mode = "monitor";
	int delay = 100000;
	std::string port = "/dev/ttyAMA0";
	std::string partnumber = "";
	std::string report = "all";
	bool port_defined = false;
	int verbosity = 0;
	struct timeval start_time,now,last;
	gettimeofday(&now,NULL);
	gettimeofday(&last,NULL);
	gettimeofday(&start_time,NULL);
	IMUDriver::RawIMU imu_data;
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
				std::cerr << "-delay option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-r") || (arg == "--report"))
		{
			if (i + 1 < argc)
			{
				report = argv[i+1];
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--report option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-v") || (arg == "--verbose"))
		{

			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				verbosity = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--verbosity option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-pn") || (arg == "--partno"))
		{

			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				partnumber = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i]
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--partnumber option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-p") || (arg == "--port"))
		{

			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				port_defined = true;
				port = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--port option requires one argument." << std::endl;
				return 1;
			}
		}
		else
		{
		}
	}
	if(port_defined == false)
	{
		if(partnumber == PN_110012)
		{
			port = "/dev/ttyAMA0";
		}
		else if(partnumber == PN_110015)
		{
			port = "/dev/ttyACM0";
		}
	}
	IMUDriver imu;
	int status = imu.init(partnumber,port,"testdevice",verbosity);
	if(status <= 0)
	{
		printf("[IMU]: Unable to Initialize. Exiting.\n");
		return 0;
	}
	double run_time = 0.0;
	while(1)
	{
		if(mode == "monitor")
		{
			gettimeofday(&now,NULL);
			imu_data = imu.update();
			print_imudata(report,imu,imu_data);
			/*
			if((reset == false) and (run_time > 10.0))
			{
				printf("RESET\n");
				imu.reset();
				reset = true;
			}
			*/
		}
		else if(mode == "query")
		{
			printf("SN: %llu\n",(long long unsigned)imu.update().serial_number);
		}
		usleep(delay);
		run_time += (double)(delay)/1000000.0;
	}
	imu.finish();
	return 0;
}
void print_imudata(std::string report,IMUDriver driver,IMUDriver::RawIMU imu_data)
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
	char tempstr[1024];
	sprintf(tempstr,"%s[IMU]:%llu ",
			start_color.c_str(),
			(long long unsigned)imu_data.serial_number);
	if((report == "stat") or (report == "all"))
	{
		sprintf(tempstr,"%s Delay=%4.2f Seq=%d T=%4.2f U=%llu R=%4.2fHz State: %s Temp: %4.2f C",
				tempstr,
				driver.get_timedelay(),
				imu_data.sequence_number,
				imu_data.tov,
				(long long unsigned)imu_data.update_count,
				imu_data.update_rate,
				driver.map_signalstate_tostring(imu_data.signal_state).c_str(),
				(double)imu_data.temperature.value);
				
	}
	if((report == "acc") or (report == "all"))
	{
		sprintf(tempstr,"%s Acc: X=%4.2f/%d Y=%4.2f/%d Z=%4.2f/%d",
				tempstr,
				imu_data.acc_x.value,imu_data.acc_x.state,
				imu_data.acc_y.value,imu_data.acc_y.state,
				imu_data.acc_z.value,imu_data.acc_z.state);
	}
	if((report == "gyro") or (report == "all"))
	{
		sprintf(tempstr,"%s Gyro: X=%4.2f/%d Y=%4.2f/%d Z=%4.2f/%d",
				tempstr,
				imu_data.gyro_x.value,imu_data.gyro_x.state,
				imu_data.gyro_y.value,imu_data.gyro_y.state,
				imu_data.gyro_z.value,imu_data.gyro_z.state);
	}
	if((report == "mag") or (report == "all"))
	{
		sprintf(tempstr,"%s Mag: X=%4.2f/%d Y=%4.2f/%d Z=%4.2f/%d",
				tempstr,
				imu_data.mag_x.value,imu_data.mag_x.state,
				imu_data.mag_y.value,imu_data.mag_y.state,
				imu_data.mag_z.value,imu_data.mag_z.state);
	}
	sprintf(tempstr,"%s%s\n",tempstr,end_color.c_str());
	printf("%s",tempstr);

}
