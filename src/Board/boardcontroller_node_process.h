#ifndef BOARDCONTROLLERNODEPROCESS_H
#define BOARDCONTROLLERNODEPROCESS_H

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
#include <std_msgs/UInt8.h>
#include "logger.h"
#include "spimessage.h"
#include <math.h>
#include <tinyxml.h>
#define REQUIRED_ENCODER_COUNT 2
struct Message
{
	unsigned char id;
	std::string type;
	std::string name;
	uint32_t sent_counter;
	uint32_t recv_counter;
	double sent_rate;
	double recv_rate;
	bool send_me;
};

struct BoardDiagnostic
{
	uint16_t id;
	icarus_rover_v2::diagnostic diagnostic;
	double lasttime_rx;
};
class BoardControllerNodeProcess
{
public:
	struct Sensor
	{
		double tov;
		uint8_t status;
		bool initialized;
		std::string type;
		std::string name;
		std::string remapped_topicname;
		icarus_rover_v2::device connected_board;
		icarus_rover_v2::pin connected_pin;
	    bool convert;
		double value;
	    std::string units;
		std::string output_datatype;
	    double min_inputvalue;
	    double max_inputvalue;
	    double min_outputvalue;
	    double max_outputvalue;
	};

	BoardControllerNodeProcess(std::string _base_node_name,std::string _node_name);
	std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }
	~BoardControllerNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	void set_mydevice(icarus_rover_v2::device device) { mydevice = device; initialized = true; }
	icarus_rover_v2::diagnostic update(double dt);
	bool get_ready_to_arm() { return ready_to_arm; }
	double measure_timediff(struct timeval a, struct timeval b);
	icarus_rover_v2::diagnostic new_message_sent(unsigned char id);
	icarus_rover_v2::diagnostic new_message_recv(unsigned char id);
	std::string get_messageinfo(bool v);
	icarus_rover_v2::diagnostic send_querymessage(unsigned char id);
	std::vector<Message> get_querymessages_tosend();
	icarus_rover_v2::diagnostic send_commandmessage(unsigned char id);
	std::vector<Message> get_commandmessages_tosend();
	icarus_rover_v2::diagnostic get_LEDStripControlParameters(unsigned char& LEDPixelMode,unsigned char& Param1,unsigned char& Param2);
	bool is_ready() { return ready; }
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(icarus_rover_v2::command cmd);
	bool is_initialized() { return initialized; }
    icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device newdevice);
    
	//Individual message processing
	icarus_rover_v2::diagnostic new_message_TestMessageCounter(uint8_t boardid,unsigned char v1,unsigned char v2,unsigned char v3,unsigned char v4,
															   unsigned char v5,unsigned char v6,unsigned char v7,unsigned char v8,
															   unsigned char v9,unsigned char v10,unsigned char v11,unsigned char v12);
	icarus_rover_v2::diagnostic new_message_GetDIOPort1(uint8_t boardid,double tov,int16_t v1,int16_t v2);
	icarus_rover_v2::diagnostic new_message_GetANAPort1(uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,
														uint16_t v4,uint16_t v5,uint16_t v6);
	icarus_rover_v2::diagnostic new_message_Diagnostic(uint8_t boardid,unsigned char System,unsigned char SubSystem,
																unsigned char Component,unsigned char Diagnostic_Type,
																unsigned char Level,unsigned char Message);
	bool board_present(icarus_rover_v2::device device);
	bool find_capability(std::vector<std::string> capabilities,std::string name);
	Sensor find_sensor(std::string name);
	bool update_sensorinfo(Sensor sensor);
	std::vector<Sensor> get_sensordata() { return sensors; }
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	std::vector<BoardDiagnostic> get_boarddiagnostics() { return board_diagnostics; }

private:
	std::string base_node_name;
	std::string node_name;
	bool unittest_running;
    std::vector<icarus_rover_v2::diagnostic> check_program_variables();
    std::vector<icarus_rover_v2::diagnostic> run_unittest();

	void init_messages();
	std::string map_PinFunction_ToString(int function);
    double map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output);
	int map_PinFunction_ToInt(std::string Function);
	bool sensors_initialized();
	bool update_sensor(icarus_rover_v2::device,icarus_rover_v2::pin,double tov,double value);
	bool load_sensorinfo(std::string name);
	bool parse_sensorfile(TiXmlDocument doc,std::string name);
	icarus_rover_v2::device find_board(uint8_t boardid);
	icarus_rover_v2::pin find_pin(icarus_rover_v2::device board,std::string pinfunction,uint8_t pinnumber);
	icarus_rover_v2::pin find_pin(icarus_rover_v2::device board,std::string pinname);
	std::string myhostname;
	icarus_rover_v2::diagnostic diagnostic;
	double run_time;
	std::vector<Message> messages;
	icarus_rover_v2::device mydevice;
    std::vector<icarus_rover_v2::device> boards;
    std::vector<BoardDiagnostic> board_diagnostics;

    std::vector<Sensor> sensors;
    std::vector<bool> boards_running;
    bool initialized;
    bool ready;
    bool ready_to_arm;
    unsigned char LEDPixelMode;
    uint8_t encoder_count;
};
#endif
