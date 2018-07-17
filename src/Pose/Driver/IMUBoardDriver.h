#ifndef IMUBoardDriver_h
#define IMUBoardDriver_h

#include <time.h>
#include <math.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

class IMUBoardDriver
{
public:

	IMUBoardDriver();
	~IMUBoardDriver();
	void init();
	int get_address() { return address; }
private:
	int address;
};


#endif
