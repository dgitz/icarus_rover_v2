#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../Driver/TerminalHatDriver.h"
/*
    - Show all current input/output pin status's
*/
using namespace std;

static void show_usage()
{
    std::cerr   << "This program is used to test the operation of a directly connected Terminal Hat.\n"
                << "As a TerminalHat is electrically the same as a ControlModule this essentially supports any ControlModule.\n"
                << "Currently supported Part Numbers:\n"
                << "\tPN: 100003\n"
                << "Usage: Tests TerminalHat via GPIO pins. Options:\n"
                << "\t-h,--help\t\tShow this help message.\n"
                << "\t-v,--verbose\t\tSet Verbosity. Default=1\n"
                << "\t-vn,--version\t\tPrint version of wiringPi Library.\n"
                << "\t-r,--readall\t\tRead all GPIO Pins.\n"
                << "\t-view,--view\t\tPrint Pin Map.\n"
                << "\t-pn,--partnumber\tPart Number. Default=100003.\n"
                << "\t-m,--mode Pin Mode\tValid Options: DigitalInput,DigitalOutput. Default=DigitalInput.\n"
                << "\t-d,--delay Delay (uS)\tDelay in micro Seconds.  Default is 100000.\n"
                << "\t-p,--pin Pin Name\tRaspberry Pi Broadcom GPIO Pin Name (ex: GPIO23).\n"
                << "\t-o,--output Pin Output\tOutput for Pin, only used for Pin Mode=DigitalOutput.  Valid Options: 0,1,toggle. Default=0.\n"
                << std::endl;
}

int main(int argc, char* argv[])
{
    std::string pinmode = "DigitalInput";
    std::string pinname = "";
    bool verbose = true;
    int delay = 100000;
    std::string partnumber = "100003";
    std::string pin_output = "0";
    TerminalHatDriver terminalhat;
    bool print_pinmap = false;
    int pin_value = 0;
    if (argc < 2) {
        show_usage();
        return 0;
    }
    for (int i = 1; i < argc; ++i) 
    {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) 
        {
            show_usage();
            return 0;
        } 
        else if ((arg == "-v") || (arg == "--verbose"))
        {
            verbose = true; 
        }
        else if ((arg == "-view") || (arg == "--view"))
        {
            print_pinmap = true;
        }
        else if ((arg == "-vn") || ( arg == "--version"))
        {
            system("gpio -v");
            return 0;
        }
        else if ((arg == "-r") || ( arg == "--readall"))
        {
            system("gpio readall");
            return 0;
        }
        else if ((arg == "-pn") || (arg == "--partnumber"))
        {
            if (i + 1 < argc) 
            { 
                partnumber = argv[i+1];
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--Part Number option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-m") || (arg == "--mode"))
        {
            if (i + 1 < argc) 
            { 
                pinmode = argv[i+1];
                if((pinmode == "DigitalInput") or (pinmode == "DigitalOutput"))
                {
                    
                }
                else { printf("Pin Mode: %s Not Supported.\n",pinmode.c_str()); }
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--Pin Mode option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-p") || (arg == "--pin"))
        {
            if (i + 1 < argc) 
            { 
                pinname = argv[i+1];
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--Pin Mode option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-o") || (arg == "--output"))
        {
            if (i + 1 < argc) 
            { 
                pin_output = argv[i+1];
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--Pin Mode option requires one argument." << std::endl;
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
    }
    bool status = terminalhat.init(partnumber);
    if(print_pinmap == true)
    {
        terminalhat.print_pinmap();
        return 0;
    }
    status = terminalhat.configure_pin(pinname,pinmode);
    if(status == false)
    {
        printf("Unable to Configure Pin: %s with mode: %s\n",pinname.c_str(),pinmode.c_str());
        return 0;
    }
    while(status)
    {
        if(pinmode == "DigitalInput")
        {
            if(verbose == true) { printf("%s V: %d\n",pinname.c_str(),terminalhat.read_pin(pinname)); }
        }
        else if(pinmode == "DigitalOutput")
        {
            if(pin_output == "0")
            {
                terminalhat.set_pin(pinname,0);
            }
            else if(pin_output == "1")
            {
                terminalhat.set_pin(pinname,1);
            }
            else if(pin_output == "toggle")
            {
                
                if(pin_value == 0) { pin_value = 1; }
                else { pin_value = 0; }
                if(verbose == true) { printf("Toggling pin: %s to value: %d\n",pinname.c_str(),pin_value); }
                terminalhat.set_pin(pinname,pin_value);
            }
        }
        usleep(delay);
    }
    return 0;
}
