#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../../include/Supported_PN.h"

using namespace std;

static void show_usage()
{
	std::cerr 	<< "This program is used to test the operation of a directly connected audio input/output device.\n"
				<< "Currently supported Part Numbers for Input:\n"
				<< "\tPN: " << PN_160002 << " (For ControlModules)\n"
				<< "\t Regular System Audio Input (For ComputeModules)\n"
				<< "Currently supported Part Numbers for Output:\n"
				<< "\tPN: " << PN_110001 << " (For ControlModules)\n"
				<< "\t Regular System Audio Output (For ComputeModules)\n"
			  	<< "Usage: Test Audio Input/Output. Options:\n"
			 	<< "\t-h,--help\t\tShow this help message\n"
				<< "\t-a,--architecture Architecture\tThe Architecture of the Host.  Options: ControlModule, ComputeModule.  Default=ComputeModule.\n"
			  	<< "\t-m,--mode Operation Mode: Options: View,RecordPlay. Default=RecordPlay.\n"
				<< std::endl;
}
/**********************************************************
Housekeeping variables
 ***********************************************************/
int results;

/**********************************************************
Declare Functions
 ***********************************************************/
double dt(struct timeval a, struct timeval b);
enum Architecture
{
	UNKNOWN = 0,
	CONTROLMODULE = 1,
	COMPUTEMODULE = 2
};
int main(int argc, char *argv[])
{
	Architecture arch = Architecture::COMPUTEMODULE;
	std::string mode = "RecordPlay";
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
		else if ((arg == "-a") || (arg == "--architecture"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				std::string host = argv[i+1];
				if(host == "ControlModule") { arch = Architecture::CONTROLMODULE; }
				else if(host == "ComputeModule") { arch = Architecture::COMPUTEMODULE; }
				else { printf("Architecture: %s Not Supported. Exiting.\n",host.c_str()); return 0; }
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--address option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-m") || (arg == "--mode"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				mode = argv[i+1];
				if((mode == "View") ||
				   (mode == "RecordPlay"))
				   {

				   }
				else
				{
					printf("Mode: %s Not Supported. Exiting.\n",mode.c_str());
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
	if(mode == "View")
	{
		printf("--- Audio Input Devices --- \n");
		system("arecord -l");
	}
	else if(mode == "RecordPlay")
	{
		printf("[WARN]: Make sure to unmute/increase Volume!\n");
		usleep(3000000);
		printf("--- Recording Audio Sample ---\n");
		if(arch == Architecture::COMPUTEMODULE)
		{
			system("rm /tmp/audiorecord_test.wav");
			system("arecord -q -d 10 -r 16000 /tmp/audiorecord_test.wav");
		}
		usleep(3000000);
		printf("--- Playing Back Audio ---\n");
		if(arch == Architecture::COMPUTEMODULE)
		{
			system("aplay -q /tmp/audiorecord_test.wav");
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
