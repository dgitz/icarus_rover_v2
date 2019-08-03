#ifndef SampleDriver_h
#define SampleDriver_h

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

class SampleDriver
{
public:

	SampleDriver();
	~SampleDriver();
	int init(std::string t_name);
	std::string getName() { return name; }

private:
	std::string name;

};


#endif
