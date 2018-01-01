#ifndef TOPICREMAPPERNODEPROCESS_H
#define TOPICREMAPPERNODEPROCESS_H

#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <tinyxml.h>

class TopicRemapperNodeProcess
{
public:
    struct InputChannel
    {
        std::string type;
        std::string topic;
        std::string name;
        int index; 
        double minvalue;
        double maxvalue;
    };
    struct OutputChannel
    {
        std::string type;
        std::string topic;
        std::string parentdevice;
        int pinnumber;
        std::string function;
        double maxvalue;
        double minvalue;
        double neutralvalue;
        double deadband;
        double value;
    };
    struct OutputMode
    {
        std::string mode;
        std::string type;
        std::string topic;
        std::string name;
        int index;
        int required_value;
    };
    struct TopicMap
    {
        OutputMode outputmode;
        InputChannel in;
        std::vector<OutputChannel> outs;
        //OutputChannel out;
        ros::Subscriber sub;
        //ros::Publisher pub;
        std::vector<ros::Publisher> pubs;
    };

	TopicRemapperNodeProcess();
	~TopicRemapperNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	void set_diagnostic(icarus_rover_v2::diagnostic v) { diagnostic = v; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	double get_runtime() { return run_time; }
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	void set_mydevice(icarus_rover_v2::device device) { mydevice = device; initialized = true; }
	bool get_initialized() { return initialized; }
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(icarus_rover_v2::command cmd);
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();
    
    icarus_rover_v2::diagnostic load(std::string topicmapfilepath);
    std::string print_topicmaps();
    std::vector<TopicMap> get_topicmaps() { return TopicMaps; }
    void set_topicmap_sub(std::size_t i,ros::Subscriber sub);
    icarus_rover_v2::diagnostic new_joymsg(sensor_msgs::Joy joy,std::string topic);
    std::vector<icarus_rover_v2::pin> get_outputs_pins();
    std::vector<std_msgs::Float32> get_outputs_float32();
    
    
    
private:
    int parse_topicmapfile(TiXmlDocument doc);
    double scale_value(double in_value,double neutral_value,double in_min,double in_max,double out_min,double out_max, double deadband);
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    
    std::vector<TopicMap> TopicMaps;
};
#endif
