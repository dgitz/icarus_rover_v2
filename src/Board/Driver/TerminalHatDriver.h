#ifndef TerminalHatDriver_h
#define TerminalHatDriver_h

#include <time.h>
#include <math.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

class TerminalHatDriver
{
public:

	TerminalHatDriver();
	~TerminalHatDriver();
	void init();
    bool configure_pin(int pinnumber,std::string mode);
	int get_address() { return address; }
    //int map_pinfile_to_connectorpin(std::string);
    int map_connectorpin_to_pinfile(int number);
    int read_pin(int pinnumber);
    bool set_pin(int pinnumber, int v);
private:
	int address;
};


#endif
