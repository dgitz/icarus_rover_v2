#ifndef BOARDCONTROLLERNODEPROCESS_H
#define BOARDCONTROLLERNODEPROCESS_H
/*
TODO:
PPS Transmit/Receive
*/
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
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <sys/time.h>
#define INITIAL_TIMEOUT_VALUE_MS 1000
#define DIO_PORT_SIZE 8 //Number of DIO pins in 1 port
#define ANA_PORT_SIZE 4 //Number of ANA pins in 1 port
using std::string;
using namespace std;
struct Port_Info{
		int ShieldID;
		int PortID;
		bool Updated;
		std::vector<int> Available;
		std::vector<float> Value;
		std::vector<int> Mode;
		std::vector<int> Number;
		std::vector<int> DefaultValue;
        std::vector<int> ADCResolution;
        std::vector<float> VoltageReference;
        std::vector<float> ScaleFactor;
		std::vector<std::string> ConnectingDevice;
	};
struct state_ack
{
	std::string name;
	bool state;
	bool trigger;
	bool retrying;
	struct timeval orig_send_time;
	struct timeval retry_send_time;
	uint16_t retries;
	uint16_t timeout_counter;
	bool retry_mode;
	bool failed;
	int flag1;  //Various Purposes
	double stream_rate;
};
struct message_info
{
	int id;
	std::string name;
	std::string protocol;
	struct timeval last_time_received;
	struct timeval last_time_transmitted;
	long received_counter;
	long sent_counter;
	double received_rate;
	double transmitted_rate;
	double initial_time_received;
	double initial_time_transmitted;

};
class BoardControllerNodeProcess
{
public:


	BoardControllerNodeProcess();
	~BoardControllerNodeProcess();
	BoardControllerNodeProcess(std::string loc, int v);
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname,int v_id);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command commandmsg);
	icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin pinmsg);
    icarus_rover_v2::diagnostic new_diagnosticmsg(icarus_rover_v2::diagnostic diagnosticmsg);
    icarus_rover_v2::diagnostic new_pps_transmit();
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	//std::vector<int> get_portlist(int ShieldID);
    int get_dioportcount(int ShieldID);
    int get_anaportcount(int ShieldID);
	bool is_finished_initializing(){ return all_device_info_received; }
	bool initialize_Ports();
    bool get_ready_to_arm() { return ready_to_arm; }
    std::vector<icarus_rover_v2::pin> get_pins(uint8_t pin_mode);
	std::vector<Port_Info> get_alldioports() { return my_dioports; }
    std::vector<Port_Info> get_allanaports() { return my_anaports; }
	Port_Info get_dioPortInfo(int ShieldID,int PortID)
	{
		for(int i = 0; i < my_dioports.size();i++)
		{
			if((my_dioports.at(i).ShieldID == ShieldID) && (my_dioports.at(i).PortID == PortID))
			{
				return my_dioports.at(i);
			}
		}
		Port_Info emptyport;
		emptyport.PortID = 0;
		emptyport.ShieldID = 0;
		return emptyport;
	}
    Port_Info get_anaPortInfo(int ShieldID,int PortID)
	{
		for(int i = 0; i < my_anaports.size();i++)
		{
			if((my_anaports.at(i).ShieldID == ShieldID) && (my_anaports.at(i).PortID == PortID))
			{
				return my_anaports.at(i);
			}
		}
		Port_Info emptyport;
		emptyport.PortID = 0;
		emptyport.ShieldID = 0;
		return emptyport;
	}
	//Port_Info get_PortInfo(std::string BoardName,std::string PortName);
	std::string map_PinFunction_ToString(int function);
	int map_PinFunction_ToInt(std::string Function);
	int map_DeviceType_ToInt(std::string devicetype);
	int get_nodestate() { return node_state; }
	void set_nodestate(int mode) { node_state = mode; }
	int get_boardstate() { return board_state; }
	void set_boardstate(int mode) { board_state = mode; } //Only for unit_testing
	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
    void set_nodefirmware(icarus_rover_v2::firmware fw) { node_firmware = fw; }
	bool checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers);
	icarus_rover_v2::diagnostic new_serialmessage_TestMessageCounter(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_FirmwareVersion(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Diagnostic(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_ANA_Port(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_DIO_Port(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_Mode(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_UserMessage(int packet_type,unsigned char* inpacket);
    icarus_rover_v2::diagnostic new_serialmessage_PPS(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
    icarus_rover_v2::firmware get_boardfirmware() { return board_firmware; }
    bool get_firmwarereceived() { return firmware_received; }
	void transmit_armedstate();
	icarus_rover_v2::diagnostic new_armedstatemsg(uint8_t v);
	std::vector<message_info> get_allmessage_info() { return messages; }
	state_ack get_stateack(std::string name);
	bool set_stateack(state_ack stateack);
	std::string map_mode_ToString(int mode);
	uint8_t get_armedstate() { return armed_state; }
	uint8_t get_armedcommand() { return armed_command; }
	uint8_t get_boardid() { return id; }
	int get_usbdevice_id() { return UsbDevice_id; }
	std::string get_boardname() { return my_boardname; }
	double get_runtime() { return run_time; }
    double compute_delay(uint8_t rx_id);
    double measure_time_diff(struct timeval timer_a, struct timeval timer_b);
    double get_delay_sec() { return current_delay_sec; }
    std::vector<icarus_rover_v2::device> get_shields() { return myshields; 	}
    std::vector<icarus_rover_v2::diagnostic> get_diagnostics_to_send() { return diagnostics_to_send; }
    
protected:
	state_ack send_configure_DIO_Ports;
    state_ack send_configure_ANA_Ports;
	state_ack send_defaultvalue_DIO_Port;
	state_ack send_testmessage_command;
	state_ack send_nodemode;
	state_ack send_set_DIO_Port;
	state_ack send_armedcommand;
	state_ack send_armedstate;
    state_ack send_diagnostic;
    state_ack send_pps;
    state_ack send_rovercommand;
private:

	std::string location;
	int id;
	std::string my_boardname;
	int UsbDevice_id;
	int board_state;
	int node_state;
	int prev_node_state;
	SerialMessageHandler *serialmessagehandler;
    icarus_rover_v2::firmware node_firmware;
    icarus_rover_v2::firmware board_firmware;
    bool firmware_received;
	bool configure_port(int ShieldID,std::vector<icarus_rover_v2::pin> pins);
	//bool configure_pin(std::string ShieldName,int PortID, uint8_t Number, std::string Function,std::string ConnectedDevice,uint8_t defaultvalue);
	int transducer_model(int mode,std::string SensorName,double input);
	void initialize_stateack_messages();
	void initialize_message_info();
	bool gather_message_info(int id, std::string mode);
	std::string myhostname;
	std::string sensor_spec_path;
	bool all_shield_info_received;
	bool all_sensor_info_received;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	std::vector<icarus_rover_v2::device> myshields;
	int shield_count;
    int sensor_count;
	std::vector<Port_Info> my_dioports;
    std::vector<Port_Info> my_anaports;
	bool extrapolate_values;
	long ms_timer;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
    bool ready_to_arm;
	uint8_t armed_state;
	uint8_t armed_command;
	bool enable_actuators;
	bool last_enable_actuators;
	double time_diff(struct timeval timea,struct timeval timeb);
	double run_time;
	double find_slope(std::vector<double> x,std::vector<double> y);
	double find_intercept(double slope,std::vector<double> x,std::vector<double> y);
    std::vector<icarus_rover_v2::diagnostic> diagnostics_to_send;
    uint8_t pps_counter;
    double current_delay_sec;
    bool received_arm_command;

	std::vector<message_info> messages;
    std::vector<struct timeval> pps_history;
};
#endif
