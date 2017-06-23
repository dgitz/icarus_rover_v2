#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

using namespace std;

static void show_usage(std::string name)
{
    std::cerr << "Usage: Test Timing. Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << std::endl;
}

int main(int argc, char* argv[])
{
    std::string mode = "normal";
    /*if (argc < 2) {
        show_usage(argv[0]);
        return 1;
    }
    */
    for (int i = 1; i < argc; ++i) 
    {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) 
        {
            show_usage(argv[0]);
            return 0;
        } 
        /*
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
        */
    }
	if(mode == "normal")
    {
        struct timeval now;
		while(1)
        {
            gettimeofday(&now,NULL);
            double now_sec = double(now.tv_sec) + double(now.tv_usec)/1000000.0;
            printf("Now: %f\n",now_sec);
            usleep(1000);
        }
    }
	return 0;
}
