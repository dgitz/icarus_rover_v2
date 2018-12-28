#include "IMUDriver.h"
IMUDriver::IMUDriver()
{
	debug_mode = 0;
	time_delay = 0.0;
    supported_connection_methods.push_back("serial");
    imu_data.signal_state = SIGNALSTATE_INITIALIZING;
	conn_fd = -1;
    gettimeofday(&now,NULL);
    gettimeofday(&last,NULL);
    imu_data.update_count = 0;
    imu_data.update_rate = 0.0;
    imu_data.tov = 0.0;
    imu_data.acc_x = 0.0;
    imu_data.acc_y = 0.0;
    imu_data.acc_z = 0.0;
    imu_data.gyro_x = 0.0;
    imu_data.gyro_y = 0.0;
    imu_data.gyro_z = 0.0;
    imu_data.mag_x = 0.0;
    imu_data.mag_y = 0.0;
    imu_data.mag_z = 0.0;
}
IMUDriver::~IMUDriver()
{

}


int IMUDriver::init(std::string t_connection_method,std::string t_port,std::string t_baudrate)
{
	gettimeofday(&last_timeupdate,NULL);
    connection_method = t_connection_method;
    if(connection_method == "serial")
    {
        port = t_port;
        baudrate = t_baudrate;
        conn_fd = open(port.c_str(),O_RDWR | O_NOCTTY);
        if(conn_fd < 0)
        {
            printf("[IMU]: Can't open: %s\n",port.c_str());
            return -1;
        }
        struct termios tty;
        memset(&tty,0,sizeof tty);
        if(tcgetattr(conn_fd,&tty) != 0 )
        {
            std::cout << "[IMU]: Error: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        }
        if(baudrate == "115200")
        {
            cfsetospeed(&tty,(speed_t)B115200);
            cfsetispeed(&tty,(speed_t)B115200);
        }
        else if(baudrate == "9600")
        {
            cfsetospeed(&tty,(speed_t)B9600);
            cfsetispeed(&tty,(speed_t)B9600);
        }
        else
        {
            printf("[IMU]: Baudrate: %s Not Supported.\n",baudrate.c_str());
            return false;
        }
        tty.c_cflag     &=  ~PARENB;            // Make 8n1
        tty.c_cflag     &=  ~CSTOPB;
        tty.c_cflag     &=  ~CSIZE;
        tty.c_cflag     |=  CS8;

        tty.c_cflag     &=  ~CRTSCTS;           // no flow control
        tty.c_cc[VMIN]   =  1;                  // read doesn't block
        tty.c_cc[VTIME]  =  10;                  // 0.5 seconds read timeout
        tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

        cfmakeraw(&tty);
        sleep(2); //required to make flush work, for some reason
        tcflush(conn_fd,TCIOFLUSH);
        if ( tcsetattr ( conn_fd, TCSANOW, &tty ) != 0) 
        {
            std::cout << "[IMU]: Error " << errno << " from tcsetattr" << std::endl;
            return false;
        }
    }
    else
    {
        printf("[IMU]: Unsupported Connection Method: %s\n",t_connection_method.c_str());
        printf("[IMU]: Supported Connection Methods:\n");
        for(std::size_t i = 0; i < supported_connection_methods.size(); ++i)
        {
            printf("\t[%d]%s\n",(int)i,supported_connection_methods.at(i).c_str());
        }
        conn_fd = -1;
    }
	return conn_fd;
}
int IMUDriver::finish()
{
	close(conn_fd);
	return 1;
}
IMUDriver::RawIMU IMUDriver::update()
{
    RawIMU t_imu = imu_data;
    gettimeofday(&now,NULL);
    if(connection_method == "serial")
    {   
    	double v = measure_time_diff(now,last_timeupdate);
    	if(measure_time_diff(now,last_timeupdate) > 10.0)
    	{
    		char tempstr[32];
    		sprintf(tempstr,"$T%4.2f*",convert_time(now));
    		int count = write(conn_fd,tempstr,strlen(tempstr));
    		gettimeofday(&last_timeupdate,NULL);
    	}

        std::string data = read_serialdata();
        tcflush(conn_fd,TCIOFLUSH);
        bool status = true;
        if(data.size() <= 4) //Definitely not enough characters.
        {
            status = false;
        }
        std::vector<std::string> items;
        boost::split(items,data,boost::is_any_of(","));
        if((items.size() != 10) and (status == true))
        {
            status = false;
        }
        if(status == true)
        {
            std::string start = items.at(0);
            std::string start_char = start.substr(0,1);
            if(start_char.compare("$") == 0)
            {
                std::string end = items.at(items.size()-1);
                std::string end_char = end.substr(end.size()-2,1);
                if(end_char.compare("*") == 0) //This is a complete string
                {
                    start = start.substr(1);
                    end = end.substr(0,end.size()-2);
                    t_imu.tov = std::atof(start.c_str());
                    t_imu.acc_x = std::atof(items.at(1).c_str());
                    t_imu.acc_y = std::atof(items.at(2).c_str());
                    t_imu.acc_z = std::atof(items.at(3).c_str());
                    t_imu.gyro_x = std::atof(items.at(4).c_str());
                    t_imu.gyro_y = std::atof(items.at(5).c_str());
                    t_imu.gyro_z = std::atof(items.at(6).c_str());
                    t_imu.mag_x = std::atof(items.at(7).c_str());
                    t_imu.mag_y = std::atof(items.at(8).c_str());
                    t_imu.mag_z = std::atof(end.c_str());
                    status = true;
                    
                }
                else
                {
                    status = false;
                }
            }
            else
            {
                status = false;
            }
        }
        if(status == true) //parsing is ok
        {
        	time_delay = convert_time(now)-t_imu.tov;
        	if(fabs(time_delay) > 0.25)
        	{
        		t_imu.signal_state = SIGNALSTATE_INVALID;
        	}
        	else
        	{
        		t_imu.signal_state = SIGNALSTATE_UPDATED;
        	}
            t_imu.update_count++;
            t_imu.update_rate = 1.0/(measure_time_diff(now,last));
            gettimeofday(&last,NULL);
            
        }

    }
    else
    {
    }
    if(measure_time_diff(now,last) > COMM_LOSS_THRESHOLD)
    {
        t_imu.signal_state = SIGNALSTATE_INVALID;
    }
    imu_data = t_imu;
    return imu_data;
}
std::string IMUDriver::read_serialdata()
{
	int n = 0;
	int length = 0;
	unsigned char response[256];
	int spot = 0;
	unsigned char buf = '\0';
	memset(response, '\0', sizeof response);
	do
	{
		n = read( conn_fd, &buf, 1 );
		length += n;
		//sprintf( &response[spot], "%c", buf );
		response[spot] = buf;
		spot += n;
	} while(buf != '\n' &&  buf != '\r' && n > 0);
	if (n < 0)
	{
		std::cout << "[IMU]: Error reading: " << strerror(errno) << std::endl;
		return "";
	}
	else if (n == 0)
	{
		std::cout << "[IMU]: Read nothing!" << std::endl;
		return "";
	}
	else
	{
		std::stringstream ss;
		ss << response;
        raw_data = ss.str();
        if(debug_mode == 1)
        {
        	printf("[RAW]: %s\n",raw_data.c_str());
        }
		return ss.str();
	}
}
double IMUDriver::measure_time_diff(struct timeval a,struct timeval b)
{
	double a_sec = (double)a.tv_sec + (double)(a.tv_usec)/1000000.0;
	double b_sec = (double)b.tv_sec + (double)(b.tv_usec)/1000000.0;
	return a_sec-b_sec;
}
double IMUDriver::measure_time_diff(double a,double b)
{
	return a-b;
}
double IMUDriver::convert_time(struct timeval t)
{
    double time = (double)t.tv_sec + (double)(t.tv_usec)/1000000.0;
    return time;
}
std::string IMUDriver::map_signalstate_tostring(uint8_t v)
{
    switch(v)
    {
        case SIGNALSTATE_UNDEFINED:
            return "UNDEFINED";
            break;
        case SIGNALSTATE_INVALID:
            return "INVALID";
            break;
        case SIGNALSTATE_INITIALIZING:
            return "INITIALIZING";
            break;
        case SIGNALSTATE_UPDATED:
            return "UPDATED";
            break;
        case SIGNALSTATE_HOLD:
            return "HOLD";
            break;
        case SIGNALSTATE_CALIBRATING:
            return "CALIBRATING";
            break;
        default:
            return "UNDEFINED";
            break;
    }
}
