#ifndef LCDDriver_h
#define LCDDriver_h

#include <linux/i2c-dev.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), */
#include <sys/ioctl.h>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string>

class LCDDriver
{
public:


	LCDDriver();
	~LCDDriver();
	int init(int _width, int _height);
	int set_backlightred(int v);
	int set_backlightgreen(int v);
	int set_backlightblue(int v);
	int send(std::string buffer);
	int test_comm(int v);
    bool get_initialized() { return initialized; }

private:
	int map_value(int v, int min, int max);
	int fd;
	int width;
	int height;
	int buffer_max;
	int backlight_red;
	int backlight_green;
	int backlight_blue;
    bool initialized;
    bool locked;

};


#endif
