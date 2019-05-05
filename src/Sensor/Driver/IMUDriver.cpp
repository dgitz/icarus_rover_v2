#include "IMUDriver.h"
IMUDriver::IMUDriver()
{
	conn_timeout = 0.0;
	debug_mode = 0;
	time_delay = 0.0;
	last_sequence_number = 0;
	timesync_tx_count = 0;
	supported_connection_methods.push_back("serial");
	supported_connection_methods.push_back("AHRS");
	imu_data.signal_state = SIGNALSTATE_INITIALIZING;
	conn_fd = -1;
	gettimeofday(&now,NULL);
	gettimeofday(&last,NULL);
	imu_data.sequence_number = 0;
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


int IMUDriver::init(std::string t_partnumber,std::string t_port)
{
	partnumber = map_pn_toenum(t_partnumber);
	if(partnumber == UNKNOWN)
	{
		printf("ERROR: PN: %s Not Supported.\n",t_partnumber.c_str());
		return -1;
	}

	switch(partnumber)
	{
	case PN_110013:
		connection_method = "serial";
		baudrate = "115200";
		break;
	case PN_110015:
		connection_method = "AHRS";
		baudrate = "";
		break;
	}
	gettimeofday(&last_timeupdate,NULL);
	if(connection_method == "serial")
	{
		port = t_port;
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
		if(baudrate == "57600")
		{
			cfsetospeed(&tty,(speed_t)B57600);
			cfsetispeed(&tty,(speed_t)B57600);
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
	else if(connection_method == "AHRS")
	{
		device = new AHRS(t_port,AHRS::SerialDataType::kRawData,200);
		usleep(1000000);
		if(device->IsConnected())
		{
			conn_fd= 1;
		}
		else
		{
			conn_fd= 0;
		}
	}
	else
	{
		printf("[IMU]: Supported Connection Methods:\n");
		for(std::size_t i = 0; i < supported_connection_methods.size(); ++i)
		{
			printf("\t[%d]%s\n",(int)i,supported_connection_methods.at(i).c_str());
		}
		conn_fd = -1;
	}
	//Initialize Sensor
	/*
	 *  protocol_buffer[0] = PACKET_START_CHAR;
        protocol_buffer[1] = MSGID_STREAM_CMD;

        // Data
        protocol_buffer[STREAM_CMD_STREAM_TYPE_INDEX] = stream_type;
        // convert update_rate_hz to two ascii bytes
        sprintf(&protocol_buffer[STREAM_CMD_UPDATE_RATE_HZ_INDEX], "%02X", update_rate_hz);

        // Footer
        encodeTermination( protocol_buffer, STREAM_CMD_MESSAGE_LENGTH, STREAM_CMD_MESSAGE_LENGTH - 4 );
	 *
	 */
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

	t_imu.updated = false;
	gettimeofday(&now,NULL);
	bool status = true;
	if(partnumber == PN_110013)
	{
		if((measure_time_diff(now,last_timeupdate) > 100.0) or (timesync_tx_count == 0))
		{
			if(partnumber == PN_110013)
			{
				char tempstr[32];
				sprintf(tempstr,"$T%4.2f*",convert_time(now));
				int count = write(conn_fd,tempstr,strlen(tempstr));
				gettimeofday(&last_timeupdate,NULL);
				timesync_tx_count++;
			}
		}

		std::string data = read_serialdata();
		tcflush(conn_fd,TCIOFLUSH);
		status = true;
		if(data.size() <= 4) //Definitely not enough characters.
		{
			status = false;
		}
		std::vector<std::string> items;
		boost::split(items,data,boost::is_any_of(","));
		if((items.size() != 11) and (status == true))
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
					last_sequence_number = imu_data.sequence_number;
					t_imu.sequence_number = std::atoi(items.at(1).c_str());

					t_imu.acc_x = std::atof(items.at(2).c_str());
					t_imu.acc_y = std::atof(items.at(3).c_str());
					t_imu.acc_z = std::atof(items.at(4).c_str());
					t_imu.gyro_x = std::atof(items.at(5).c_str());
					t_imu.gyro_y = std::atof(items.at(6).c_str());
					t_imu.gyro_z = std::atof(items.at(7).c_str());
					t_imu.mag_x = std::atof(items.at(8).c_str());
					t_imu.mag_y = std::atof(items.at(9).c_str());
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
	}
	else if(partnumber == PN_110015)
	{
		status = false;


		if(device->IsConnected() == true)
		{
			t_imu.tov = convert_time(now);
			last_sequence_number = imu_data.sequence_number;
			t_imu.sequence_number=device->GetUpdateCount();
			t_imu.acc_x = G*device->GetRawAccelX();
			t_imu.acc_y = G*device->GetRawAccelY();
			t_imu.acc_z = G*device->GetRawAccelZ();
			t_imu.gyro_x = device->GetRawGyroX();
			t_imu.gyro_y = device->GetRawGyroY();
			t_imu.gyro_z = device->GetRawGyroZ();
			t_imu.mag_x = device->GetRawMagX();
			t_imu.mag_y = device->GetRawMagY();
			t_imu.mag_z = device->GetRawMagZ();
			status = true;
		}
	}

	if(status == true) //parsing is ok
	{
		gettimeofday(&now,NULL);
		time_delay = convert_time(now)-t_imu.tov;
		if(fabs(time_delay) > 0.25)
		{
			t_imu.signal_state = SIGNALSTATE_INVALID;
		}
		else if(t_imu.sequence_number == last_sequence_number)
		{
			t_imu.signal_state = SIGNALSTATE_HOLD;
		}
		else
		{
			gettimeofday(&last_timeupdate,NULL);
			t_imu.signal_state = SIGNALSTATE_UPDATED;
		}
		t_imu.update_count++;
		t_imu.update_rate = 1.0/(measure_time_diff(now,last));
		gettimeofday(&last,NULL);

	}

	gettimeofday(&now,NULL);
	if(measure_time_diff(now,last) > COMM_LOSS_THRESHOLD)
	{
		t_imu.signal_state = SIGNALSTATE_INVALID;
	}
	if(measure_time_diff(now,last_timeupdate) > COMM_LOSS_THRESHOLD)
	{
		t_imu.signal_state = SIGNALSTATE_INVALID;
		gettimeofday(&last_timeupdate,NULL);
		device->ResetConnection();
	}
	t_imu.updated = true;
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
std::string IMUDriver::map_pn_tostring(IMUDriver::PartNumber v)
{
	switch(v)
	{
	case PN_110013:
		return "110013";
		break;
	case PN_110015:
		return "110015";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}
IMUDriver::PartNumber IMUDriver::map_pn_toenum(std::string v)
{
	if(v == "110013")
	{
		return PN_110013;
	}
	else if(v == "110015")
	{
		return PN_110015;
	}
	else
	{
		return UNKNOWN;
	}
}
