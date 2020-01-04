#ifndef TerminalHatDriver_h
#define TerminalHatDriver_h
#include <wiringPi.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>

class TerminalHatDriver
{
public:
	TerminalHatDriver();
	~TerminalHatDriver();
	bool init(std::string partnumber_);
    bool configure_pin(std::string pinname,std::string mode);
	int get_address() { return address; }
    
    int read_pin(std::string pinname);
    bool set_pin(std::string pinname,int v);
    void print_pinmap();
    
private:
    std::vector<std::string> supported_partnumbers;
    std::string partnumber;
    void init_pinmap();
    std::map<std::string,int> PinMap;
	int address;
};


#endif
