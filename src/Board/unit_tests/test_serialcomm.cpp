#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string.h>
#include <sys/time.h>
 #include <stdint.h>
#include <vector>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include "Definitions.h"
#include "../../../include/serialmessage.h"
//#include "../Driver/ServoHatDriver.h"

using namespace std;
double measure_timediff(timeval a, timeval b);

struct Message
{
	uint8_t id;
	uint64_t rx_counter;
	double rx_rate;
	int packet_length;
};
long long good_packet_counter;
long long bad_packet_counter;
std::vector<Message> build_messages();
std::vector<Message> decode_ROSMessage(SerialMessageHandler* handler,unsigned char* packet,int length,std::vector<Message> messages);
static void show_usage(std::string name)
{
    std::cerr << "Usage: Test ServoHat. Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-m,--mode Mode\traw,stat,send=<message>\n"
			  << "\t-p,--protocol Protocol:\tNone (Default),ROS\n"
              << "\t-d,--device SerialDevice.\n"
              << "\t-b,--baudrate BaudRate.\n"
			  //<< "\t-o,--output: Screen,"
              << std::endl;
}

int main(int argc, char* argv[])
{
	bad_packet_counter = 0;
	good_packet_counter = 0;
    std::string mode = "";
    std::string device = "";
    std::string baudrate= "";
    std::string protocol = "";
    std::vector<uint8_t> send_messages;
    std::vector<Message> messages;
    if (argc < 2) {
        show_usage(argv[0]);
        return 1;
    }
    for (int i = 1; i < argc; ++i) 
    {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) 
        {
            show_usage(argv[0]);
            return 0;
        } 
        else if ((arg == "-m") || (arg == "--mode"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                mode = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-d") || (arg == "--device"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                device = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-b") || (arg == "--baudrate"))
        {
            if (i + 1 < argc) 
            { 
                // Make sure we aren't at the end of argv!
                baudrate = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
                i++;
            } 
            else 
            {
                // Uh-oh, there was no argument to the destination option.
                std::cerr << "--address option requires one argument." << std::endl;
                return 1;
            }  
        }
        else if ((arg == "-p") || (arg == "--protocol"))
        {
        	if (i + 1 < argc)
        	{
        		// Make sure we aren't at the end of argv!
        		protocol = argv[i+1]; // Increment 'i' so we don't get the argument as the next argv[i
        		i++;
        	}
        	else
        	{
        		// Uh-oh, there was no argument to the destination option.
        		std::cerr << "--address option requires one argument." << std::endl;
        		return 1;
        	}
        }
    }
    if(protocol == "ROS")
    {
    	messages = build_messages();
    }
    std::size_t found_send = mode.find("send=");
    if(found_send != std::string::npos)
    {
    	std::string tempstr = mode.substr(5,mode.size());
    	std::vector<std::string> strs;
    	boost::split(strs,tempstr,boost::is_any_of(","));
    	for(std::size_t i = 0; i < strs.size(); i++)
    	{
    		send_messages.push_back(std::atoi(strs.at(i).c_str()));
    	}
    	mode = "send";
    }
    if((mode == "raw") || (mode == "send") || (mode == "stat"))
    {
    	printf("Testing Serial Comm with: device=%s baudrate=%s mode=%s\n",device.c_str(),baudrate.c_str(),mode.c_str());
    	SerialMessageHandler *serialmessagehandler = new SerialMessageHandler;
    	int dev_fd = open(device.c_str(),O_RDWR | O_NOCTTY);
    	if(dev_fd < 0)
    	{
    		printf("Unable to open port. Exiting.\n");
    		return -1;
    	}
    	struct termios tty;
    	memset(&tty,0,sizeof tty);
    	if(tcgetattr(dev_fd,&tty) != 0 )
    	{
    		std::cout << "Error: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    	}
    	if(baudrate == "115200")
    	{
    		cfsetospeed(&tty,(speed_t)B115200);
    		cfsetispeed(&tty,(speed_t)B115200);
    	}
    	else
    	{
    		printf("Baudrate: %s Not Supported.\n",baudrate.c_str());
    	}
    	/* Setting other Port Stuff */
    	tty.c_cflag     &=  ~PARENB;            // Make 8n1
    	tty.c_cflag     &=  ~CSTOPB;
    	tty.c_cflag     &=  ~CSIZE;
    	tty.c_cflag     |=  CS8;

    	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    	tty.c_cc[VMIN]   =  1;                  // read doesn't block
    	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    	/* Make raw */
    	cfmakeraw(&tty);
    	tcflush( dev_fd, TCIFLUSH );
    	if ( tcsetattr ( dev_fd, TCSANOW, &tty ) != 0) {
    		std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    	}
    	int packet_counter = 0;
    	long rx_counter = 0;
    	timeval now,last;
    	gettimeofday(&now,NULL);
    	last = now;
    	double run_time = 0.0;
    	double medium_timer = 0.0;
    	double slow_timer = 0.0;

    	while(1)
    	{
    		gettimeofday(&now,NULL);
    		double dt = measure_timediff(now,last);
    		run_time += dt;
    		medium_timer += dt;
    		slow_timer += dt;
    		int n = 0;
    		int length = 0;
    		unsigned char response[64];
    		int spot = 0;
    		unsigned char buf = '\0';
    		memset(response, '\0', sizeof response);
    		do
    		{
    			n = read( dev_fd, &buf, 1 );
    			//printf("%c",buf);
    			length += n;
    			rx_counter += n;
    			//sprintf( &response[spot], "%c", buf );
    			response[spot] = buf;
    			spot += n;
    		} while(buf != '\n' && buf != '\r' && n > 0);
    		//printf("read: %d\n",length);
    		if (n < 0)
    		{
    			std::cout << "Error reading: " << strerror(errno) << std::endl;
    		}
    		else if (n == 0)
    		{
    			std::cout << "Read nothing!" << std::endl;
    		}
    		else
    		{
    			packet_counter++;
    			if((mode=="raw"))
    			{
    				if(protocol == "ROS")
    				{
    					for(int i = 0; i < length; i++)
    					{
    						printf("%0x",response[i]);
    						printf(" ");
    					}
    					printf("\n");
    				}
    				else
    				{
    					printf("%s\n",response);
    				}

    			}
    			if((protocol == "ROS"))
    			{
    				messages = decode_ROSMessage(serialmessagehandler,response,length,messages);
    			}

    		}

    		if(medium_timer > 0.25)
    		{
    			medium_timer = 0.0;
    			if((mode=="stat") || (mode == "send"))
    			{
    				for(std::size_t i = 0; i < messages.size(); i++)
    				{
    					messages.at(i).rx_rate = (double)(messages.at(i).rx_counter)/run_time;
    				}
    				printf("Total Rx: %d @ Rate: %f bps: %f Bpp: %f\n",packet_counter,(double)(packet_counter)/run_time,8.0*(double)(rx_counter)/run_time,(double)(rx_counter)/(double)(packet_counter));
    				double ratio = 100.0*(double)(bad_packet_counter)/((double)(bad_packet_counter)+(double)(good_packet_counter));
    				printf("Bad Packets: %lld @ Rate: %f Ratio: %0.2f%\n",bad_packet_counter,(double)(bad_packet_counter)/run_time,ratio);
    				for(std::size_t i = 0; i < messages.size(); i++)
    				{
    					if(messages.at(i).rx_counter > 0)
    					{
    						printf("Msg: 0XAB%0X Rx: %d Rate: %f Len: %d\n",messages.at(i).id,messages.at(i).rx_counter,messages.at(i).rx_rate,messages.at(i).packet_length);
    					}
    				}
    			}

    		}
    		if(slow_timer > 1.0)
    		{
    			slow_timer = 0.0;

    			if(mode == "send")
    			{
    				unsigned char buffer[64];
    				int length = 0;
    				int tx_status = 0;
    				int v1,v2,v3,v4;
    				v1 = 0;
    				v2 = 0;
    				v3 = 0;
    				v4 = 0;
    				int msg_id = send_messages.at(0);
    				switch(msg_id)
    				{
    				case SERIAL_Command_ID:
    					if(send_messages.size() >= 2) { v1 = send_messages.at(1); }
    					if(send_messages.size() >= 3) { v2 = send_messages.at(2); }
    					if(send_messages.size() >= 4) { v3 = send_messages.at(3); }
    					if(send_messages.size() >= 5) { v4 = send_messages.at(4); }
    					tx_status = serialmessagehandler->encode_CommandSerial(buffer,&length,v1,v2,v3,v4);
    					break;
    				default:
    					printf("Can't send: 0XAB%0X: Not Supported.\n",msg_id);
    					break;
    				}
    				if(tx_status)
    				{
    					int count = write( dev_fd, buffer, length );
    					//int count = write(dev_fd,reinterpret_cast<char*> (&buffer[0]),length);
    					if (count < 0)
    					{
    						printf("didn't send\n");
    					}
    					else
    					{
    						printf("sent: %d bytes\n",count);
    					}

    				}
    			}


    		}
    		last = now;
    	}
    	close(dev_fd);
    }
	return 0;
}
double measure_timediff(timeval b, timeval a)
{
    double t1 = (double)(a.tv_sec) + (double)(a.tv_usec)/1000000.0;
    double t2 = (double)(b.tv_sec) + (double)(b.tv_usec)/1000000.0;
    return t2-t1;
}
std::vector<Message> decode_ROSMessage(SerialMessageHandler* handler,unsigned char* packet,int length,std::vector<Message> messages)
{
	if(packet[0] == 0xAB)
	{
		int id = packet[1];
		int packet_length = (int)packet[2];
		if((length-packet_length) != 5)
		{
			bad_packet_counter++;
			return messages;
		}
		int computed_checksum = 0;
		for(int i = 3; i < length-2; i++)
		{
			computed_checksum ^= packet[i];
		}
		if(computed_checksum != packet[length-2])
		{
			bad_packet_counter++;
			return messages;
		}
		unsigned char uchar1,uchar2,uchar3,uchar4;
		unsigned long ulong1,ulong2,ulong3,ulong4;
		long long1,long2,long3,long4,long5,long6,long7,long8,long9;
		int int1;
		switch(id)
		{
			case SERIAL_UserMessage_ID:
				break;
			case SERIAL_Command_ID:
				handler->decode_CommandSerial(packet,&uchar1,&uchar2,&uchar3,&uchar4);
				//printf("c1: %d c2: %d c3: %d c4: %d\n",uchar1,uchar2,uchar3,uchar4);
				break;
			case SERIAL_Diagnostic_ID:
				break;
			case SERIAL_TestMessageCommand_ID:
				break;
			case SERIAL_Configure_DIO_Port_ID:
				break;
			case SERIAL_Mode_ID:
				break;
			case SERIAL_Set_DIO_Port_ID:
				break;
			case SERIAL_FirmwareVersion_ID:
				break;
			case SERIAL_Arm_Status_ID:
				break;
			case SERIAL_Set_DIO_Port_DefaultValue_ID:
				break;
			case SERIAL_PPS_ID:
				break;
			case SERIAL_Configure_ANA_Port_ID:
				break;
			case SERIAL_ID_ID:

				handler->decode_IDSerial(packet,&uchar1,&ulong1);
				//printf("c: %d l: %ld\n",uchar1,ulong1);
				break;
			//case SERIAL_IMU_ID:
			//	handler->decode_IMUSerial(packet,&ulong1,&int1,&long1,&long2,&long3,&long4,&long5,&long6,&long7,&long8,&long9);
				//printf("t: %ld count: %d ax: %ld ay: %ld az: %ld gx: %ld gy: %ld gz: %ld mx: %ld my: %ld mz: %ld\n",
				//		ulong1,int1,long1,long2,long3,long4,long5,long6,long7,long8,long9);
				break;
			default:
				bad_packet_counter++;
				printf("Packet ID: 0XAB%0x Not Supported.\n",id);
				return messages;
				break;

		}
		good_packet_counter++;
		for(std::size_t i = 0; i < messages.size(); i++)
		{
			if(id == messages.at(i).id)
			{
				messages.at(i).rx_counter++;
				messages.at(i).packet_length = length;
			}
		}
	}

	/*
	if((packet[1] == 'A') &&
	   (packet[2] == 'B'))
	{

		return true;
	}
	*/

	return messages;
}
std::vector<Message> build_messages()
{
	std::vector<Message> messages;
	{
		Message message;
		message.id = SERIAL_UserMessage_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Command_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Diagnostic_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_TestMessageCommand_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Configure_DIO_Port_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Mode_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_FirmwareVersion_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Arm_Status_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Set_DIO_Port_DefaultValue_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_PPS_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_Configure_ANA_Port_ID;
		messages.push_back(message);
	}
	{
		Message message;
		message.id = SERIAL_ID_ID;
		messages.push_back(message);
	}

	for(std::size_t i = 0; i < messages.size(); i++)
	{
		messages.at(i).rx_counter = 0;
		messages.at(i).rx_rate = 0.0;
		messages.at(i).packet_length = 0;
	}
	return messages;

}
