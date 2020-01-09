#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../../include/Supported_PN.h"
#include "../Driver/ServoHatDriver.h"

using namespace std;

static void show_usage()
{
    std::cerr   << "This program is used to test the operation of a directly connected Servo Hat.\n"
                << "Currently supported Part Numbers:\n"
                << "\tPN: " << PN_625004 << "\n"
                << "Usage: Test ServoHat via I2C. Options:\n"
                << "\t-h,--help\t\tShow this help message\n"
                << "\t-a,--address Address\tThe I2C Address of the Hat (Base 10).  Default=64.\n"
                << "\t-r,--reset\t\tReset all PWM Outputs. Default=0.\n"
                << "\t-m,--mode\t\tMode: oneshot, sweep. Default=oneshot.\n"
                << "\toneshot: Sets a PWM Channel with a value. Options:\n"
                << "\t -c,--channel Channel\tPWM Channel.  Valid Range: 0-15. Default=0.\n"
                << "\t -v,--value Value\tPWM Value. Valid Range: 0-2000. Default=1000.\n"
                << "\tsweep: Sweeps a PWM Channel with a value between a start range and stop range. Options:\n"
                << "\t -c,--channel Channel\tPWM Channel.  Valid Range: 0-15. Default=0.\n"
                << "\t -s,--start Start\tStart PWM Value.  Valid Range: 0-2000. Default=1000.\n"
                << "\t -e,--end End\t\tStop PWM Value. Valid Range: 0-2000. Default=1000.\n"
                << "\t -i,---increment Inc\tAmount to increment sweep by.  Default: 1.\n"
                << std::endl;
}

int main(int argc, char* argv[])
{
    std::string mode = "oneshot";
    int hat_address = 64;
    int channel = 0;
    int value = 1000;
    int reset = 0;
    int start_sweep = 0;
    int stop_sweep = 2000;
    int increment = 1;
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
        else if ((arg == "-r") || (arg == "--reset"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                reset = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }         
        else if ((arg == "-c") || (arg == "--channel"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                channel = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        } 
        else if ((arg == "-v") || (arg == "--value"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                value = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        } 
        else if ((arg == "-s") || (arg == "--start"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                start_sweep = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        } 
        else if ((arg == "-e") || (arg == "--end"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                stop_sweep = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-i") || (arg == "--increment"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                increment = atoi(argv[i+1]); // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }         
        else 
        {
        }
    }
    if(mode == "sweep") {    value = start_sweep; }
    bool direction = true;
    ServoHatDriver servohat;
    int status = servohat.init(hat_address);
    printf("status: %d\n",status);
	if(reset == 1)
    {
		servohat.resetAllServo();
    }
    else
    {
        while(1)
        {
            if(mode == "oneshot")
            {
                cout << "Setting ServoHat Address: " << hat_address << " Channel: " << channel << " To Value: " << value << endl;
                servohat.setServoValue(channel,value);
                usleep(100000);  // wait for 0.1 seconds

            }
            else if(mode == "sweep")
            {
                
                //printf("value: %d direction: %d start: %d end: %d\n",value,direction,start_sweep,stop_sweep);
                if(value > stop_sweep)
                {
                    direction = !direction;
                }
                else if(value < start_sweep)
                {
                    direction = !direction;
                }
                else
                {
                    cout << "Setting ServoHat Address: " << hat_address << " Channel: " << channel << " To Value: " << value << endl;
                    servohat.setServoValue(channel,value);
                    usleep(1000);  // wait for 0.1 seconds
                }
                if(direction == true)
                {
                    value+=increment;
                }
                else
                {
                    value-=increment;
                }
                
               
            }
        }
    }
	return 0;
}
