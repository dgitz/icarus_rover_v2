#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
#include "spimessage.h"
/*! \class BoardControllerNodeProcess BoardControllerNodeProcess.h "BoardControllerNodeProcess.h"
 *  \brief This is a BoardControllerNodeProcess class.  Used for the boardcontroller_node node.
 *
 */
class BoardControllerNodeProcess: public BaseNodeProcess {
public:
	//Constants
    /*! \brief  Require 2 Encoders in DeviceFile. */
	const uint8_t REQUIRED_ENCODER_COUNT = 2;
	//Enums
	//Structs
    /*! \brief  SPI Message statistics structure */
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
    /*! \brief  Board Diagnostic statistic structure */
	struct BoardDiagnostic
	{
		uint16_t id;
		icarus_rover_v2::diagnostic diagnostic;
		double lasttime_rx;
	};
    /*! \brief Sensor info structure */
	struct Sensor
	{
		icarus_rover_v2::signal signal;
		bool initialized;
		std::string type;
		std::string name;
		std::string remapped_topicname;
		icarus_rover_v2::device connected_board;
		icarus_rover_v2::pin connected_pin;
		bool convert;
		std::string output_datatype;
		double min_inputvalue;
		double max_inputvalue;
		double min_outputvalue;
		double max_outputvalue;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization  */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);
	//Attribute Functions
	icarus_rover_v2::command get_current_command() { return current_command; }
	uint8_t get_armed_state() { return armed_state; }
	//Message Functions
	/*! \brief  Process Command Message. */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
    /*! \brief  Process Device Message. */
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
    /*! \brief  Process Armed State Message. */
	void new_armedstatemsg(uint8_t t_armed_state);
	icarus_rover_v2::diagnostic new_message_sent(unsigned char id);
	icarus_rover_v2::diagnostic new_message_recv(unsigned char id);
	std::string get_messageinfo(bool v);
    /*! \brief  Send SPI Query Message */
	icarus_rover_v2::diagnostic send_querymessage(unsigned char id);
    /*! \brief  Get list of Query Messages to Send */
	std::vector<Message> get_querymessages_tosend();
    /*! \brief Send SPI Command Message. */
	icarus_rover_v2::diagnostic send_commandmessage(unsigned char id);
    /*! \brief  Get list of Command Messages to Send */
	std::vector<Message> get_commandmessages_tosend();
    /*! \brief  Get parameters to send to LED Strip */
	icarus_rover_v2::diagnostic get_LEDStripControlParameters(unsigned char& LEDPixelMode,unsigned char& Param1,unsigned char& Param2);
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
	//Support Functions
	bool board_present(const icarus_rover_v2::device::ConstPtr& device);
	bool find_capability(std::vector<std::string> capabilities,std::string name);
	Sensor find_sensor(std::string name);
	bool update_sensorinfo(Sensor sensor);
	std::vector<Sensor> get_sensordata() { return sensors; }
	std::vector<BoardDiagnostic> get_boarddiagnostics() { return board_diagnostics; }

	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
	void init_messages();
	std::string map_PinFunction_ToString(int function);
	double map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output);
	int map_PinFunction_ToInt(std::string Function);
	bool sensors_initialized();
	bool update_sensor(const icarus_rover_v2::device::ConstPtr& t_device,const icarus_rover_v2::pin::ConstPtr& t_pin,double tov,double value);
	bool load_sensorinfo(std::string name);
	bool parse_sensorfile(TiXmlDocument doc,std::string name);
	icarus_rover_v2::device find_board(uint8_t boardid);
	icarus_rover_v2::pin find_pin(const icarus_rover_v2::device::ConstPtr& t_board,std::string pinfunction,uint8_t pinnumber);
	icarus_rover_v2::pin find_pin(const icarus_rover_v2::device::ConstPtr& t_board,std::string pinname);

	std::vector<Message> messages;
	std::vector<icarus_rover_v2::device> boards;
	std::vector<BoardDiagnostic> board_diagnostics;

	std::vector<Sensor> sensors;
	std::vector<bool> boards_running;
	unsigned char LEDPixelMode;
	uint8_t encoder_count;
	icarus_rover_v2::command current_command;
	uint8_t armed_state;
};
