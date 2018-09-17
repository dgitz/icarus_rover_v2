#include "LCDDriver.h"
LCDDriver::LCDDriver()
{
	fd = -1;
	width = 0;
	height = 0;
	buffer_max = 0;
	backlight_red = 0;
	backlight_green = 0;
	backlight_blue = 0;
	locked = false;
}
LCDDriver::~LCDDriver()
{
	close(fd);
}


int LCDDriver::init(int _width,int _height)
{
    
	width = _width;
	height = _height;
	buffer_max = width*height;
	int id = -1;
	int port = open( "/dev/ttyAMA0",O_RDWR| O_NONBLOCK | O_NDELAY );
	/* Error Handling */
	if ( port < 0 )
	{
		printf("Error opening /dev/ttyAMA0. Exiting.\n");
		return 0;
	}

	/* *** Configure Port *** */
	struct termios tty;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if ( tcgetattr ( port, &tty ) != 0 )
	{
		printf("Error getting tcgetattr. Exiting.\n");
		return 0;
	}

	/* Set Baud Rate */
	cfsetospeed (&tty, B9600);
	cfsetispeed (&tty, B9600);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;        // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;
	tty.c_cflag     &=  ~CRTSCTS;       // no flow control
	tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
	tty.c_oflag     =   0;                  // no remapping, no delays
	tty.c_cc[VMIN]      =   0;                  // read doesn't block
	tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
	tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
	tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	tty.c_oflag     &=  ~OPOST;              // make raw

	/* Flush Port, then applies attributes */
	tcflush( port, TCIFLUSH );

	if ( tcsetattr ( port, TCSANOW, &tty ) != 0)
	{
		printf("Error setting tcsetattr. Exiting.\n");
		return 0;
	}
	fd = port;
    initialized = true;
	return 1;
}
int LCDDriver::send(std::string buffer)
{
	if(locked == true)
	{
		return 0;
	}
	locked = true;
	if(buffer.size() > buffer_max)
	{
		printf("Buffer is too big: %d/%d\n",buffer.size(),buffer_max);
		return -1;
	}
	write(fd,"|-",2);
	int n_written = write (fd, buffer.c_str(), strlen(buffer.c_str()));
	locked = false;
	return n_written;
}
int LCDDriver::set_backlightred(int v)
{
	backlight_red = v;
	char tempstr[2];
	sprintf(tempstr,"|%c",(unsigned char)(map_value(v,128,157)));
    return write(fd,tempstr,2);
}
int LCDDriver::set_backlightgreen(int v)
{
	backlight_green = v;
	char tempstr[2];
	sprintf(tempstr,"|%c",(unsigned char)(map_value(v,158,187)));
	return write(fd,tempstr,2);
}
int LCDDriver::set_backlightblue(int v)
{
	backlight_blue = v;
	char tempstr[2];
	sprintf(tempstr,"|%c",(unsigned char)(map_value(v,188,217)));
	return write(fd,tempstr,2);
}
int LCDDriver::map_value(int v, int min, int max)
{
	//y-y1 = m(x-x1)
	//y = m(x-x1) + y1
	double x_min = 0.0;
	double x_max = 100.0;
	double y_min = (double)min;
	double y_max = (double)max;
	double m = (y_max-y_min)/(x_max-x_min);
	double in = (double)v;
	if(in > x_max) { in = x_max; }
	if(in < x_min) { in = x_min; }
	double value = m*(in-x_min) + y_min;
	return (int)value;
}
