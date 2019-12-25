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
                << "\tPN: 100008\n"
                << "\tPN: 100009\n"
                << "Usage: Tests TerminalHat via GPIO pins. Options:\n"
                << "\t-h,--help\t\tShow this help message.\n"
                << "\t-v,--verbose\t\tSet Verbosity.\n"
                << "\t-a,--all\t\tPrint Pin Map.\n"
                << "\t-m,--mode Pin Mode\t\tValid Options: DigitalInput,DigitalOutput. Default=DigitalInput.\n"
                << "\t-d,--delay Delay (uS)\t\tDelay in micro Seconds.  Default is 100000.\n"
                << "\t-p,--pin Pin Number\t\tRaspberry Pi 40-pin connector Pin Number.\n"
                << "\t-o,--output Pin Output\t\tOutput for Pin, only used for Pin Mode=DigitalOutput.  Valid Options: 0,1,toggle. Default=0.\n"
                << std::endl;
}

int main(int argc, char* argv[])
{
    std::string pinmode = "DigitalInput";
    int pinnumber = -1;
    bool verbose = false;
    int delay = 100000;
    std::string pin_output = "0";
    TerminalHatDriver terminalhat;
    terminalhat.init();
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
        else if ((arg == "-a") || (arg == "==all"))
        {
            terminalhat.print_pinmap();
            return 0;
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
                pinnumber = atoi(argv[i+1]);
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
    bool status = terminalhat.configure_pin(pinnumber,pinmode);
    if(status == false)
    {
        printf("Unable to Configure Pin: %d with mode: %s\n",pinnumber,pinmode.c_str());
        return 0;
    }
    while(status)
    {
        if(pinmode == "DigitalInput")
        {
            if(verbose == true) { printf("C: %d V: %d\n",pinnumber,terminalhat.read_pin(pinnumber)); }
        }
        else if(pinmode == "DigitalOutput")
        {
            if(pin_output == "0")
            {
                terminalhat.set_pin(pinnumber,0);
            }
            else if(pin_output == "1")
            {
                terminalhat.set_pin(pinnumber,1);
            }
            else if(pin_output == "toggle")
            {
                
                if(pin_value == 0) { pin_value = 1; }
                else { pin_value = 0; }
                if(verbose == true) { printf("Toggling pin: %d to value: %d\n",pinnumber,pin_value); }
                terminalhat.set_pin(pinnumber,pin_value);
            }
        }
        usleep(delay);
    }
    return 0;
}
