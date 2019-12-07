#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
#include "../../include/spimessage.h"
/*! \class BoardControllerNodeProcess BoardControllerNodeProcess.h "BoardControllerNodeProcess.h"
 *  \brief This is a BoardControllerNodeProcess class.  Used for the boardcontroller_node node.
 *
 */
class BoardControllerNodeProcess: public BaseNodeProcess {
public:
	//Constants
    /*! \brief  Require 2 Encoders in DeviceFile. */
	const uint8_t REQUIRED_ENCODER_COUNT = 2;
	const double BOARDCOMM_TIMEOUT_WARN = 5.0;
	const double BOARDCOMM_TIMEOUT_ERROR = 10.0;
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
	/*! \brief Board info structure */
	struct Board
	{
		eros::device device;
		double lasttime_rx;
	};
    /*! \brief Sensor info structure */
	struct Sensor
	{
		eros::signal signal;
		double conversion_factor;
		bool initialized;
		std::string type;
		std::string name;
		std::string remapped_topicname;
		eros::device connected_board;
		eros::pin connected_pin;
		bool convert;
		std::string output_datatype;
		double min_inputvalue;
		double max_inputvalue;
		double min_outputvalue;
		double max_outputvalue;
	};
	struct PinDefinition
	{
		std::string PinName;
		uint8_t port_id;
		uint8_t port_pinnumber;
	};
	struct BoardMap
	{
		std::string DeviceType;
		std::string FAST_PN;
		std::vector<PinDefinition> PinMap;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization  */
	eros::diagnostic finish_initialization();
	bool initialize_supportedboards();
	//Update Functions
	/*! \brief Implementation of the update function */
	eros::diagnostic update(double t_dt,double t_ros_time);
	eros::diagnostic update_pin(std::string device_type,uint8_t device_id,std::string pin_name,eros::pin new_pin);
	//Attribute Functions
	double get_boardcomm_timeout_warn_threshold() { return BOARDCOMM_TIMEOUT_WARN; }
	double get_boardcomm_timeout_error_threshold() { return BOARDCOMM_TIMEOUT_ERROR; }
	eros::command get_current_command() { return current_command; }
	uint8_t get_armed_state() { return armed_state; }
	//Message Functions
	/*! \brief  Process Command Message. */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
    /*! \brief  Process Device Message. */
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
    /*! \brief  Process Armed State Message. */
	void new_armedstatemsg(uint8_t t_armed_state);
	eros::diagnostic new_message_sent(unsigned char id);
	eros::diagnostic new_message_recv(unsigned char id);
	std::string get_messageinfo(bool v);
    /*! \brief  Send SPI Query Message */
	eros::diagnostic send_querymessage(unsigned char id);
    /*! \brief  Get list of Query Messages to Send */
	std::vector<Message> get_querymessages_tosend();
    /*! \brief Send SPI Command Message. */
	eros::diagnostic send_commandmessage(unsigned char id);
    /*! \brief  Get list of Command Messages to Send */
	std::vector<Message> get_commandmessages_tosend();
    /*! \brief  Get parameters to send to LED Strip */
	eros::diagnostic get_LEDStripControlParameters(unsigned char& LEDPixelMode,unsigned char& Param1,unsigned char& Param2);
	//Individual message processing
	eros::diagnostic new_message_TestMessageCounter(uint8_t boardid,unsigned char v1,unsigned char v2,unsigned char v3,unsigned char v4,
			unsigned char v5,unsigned char v6,unsigned char v7,unsigned char v8,
			unsigned char v9,unsigned char v10,unsigned char v11,unsigned char v12);
	eros::diagnostic new_message_GetDIOPort1(std::string device_type,uint8_t boardid,double tov,int16_t v1,int16_t v2);
	eros::diagnostic new_message_GetANAPort1(std::string device_type,uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,
			uint16_t v4,uint16_t v5,uint16_t v6);
	eros::diagnostic new_message_Diagnostic(uint8_t boardid,unsigned char System,unsigned char SubSystem,
			unsigned char Component,unsigned char Diagnostic_Type,
			unsigned char Level,unsigned char Message);
	//Support Functions
	bool board_present(const eros::device::ConstPtr& device);
	bool find_capability(std::vector<std::string> capabilities,std::string name);
	Sensor find_sensor(std::string name);
	bool update_sensorinfo(Sensor sensor);
	std::vector<Sensor> get_sensordata() { return sensors; }
	std::vector<Board> get_boarddata() { return boards; }
	std::vector<BoardMap> get_allsupportedboards() { return supported_boards; }
	BoardMap get_boardmap_bypartnumber(std::string partnumber);
	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation */
	std::vector<eros::diagnostic> check_programvariables();
	PinDefinition create_pindefinition(std::string pinname,uint8_t port_id,uint8_t port_pinnumber)
	{
		PinDefinition pin_def;
		pin_def.PinName = pinname;
		pin_def.port_id = port_id;
		pin_def.port_pinnumber = port_pinnumber;
		return pin_def;
	}
	void init_messages();
	std::string map_PinFunction_ToString(int function);
	double map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output);
	int map_PinFunction_ToInt(std::string Function);
	bool sensors_initialized();
	bool update_sensor(const eros::device::ConstPtr& t_device,const eros::pin::ConstPtr& t_pin,double tov,double value);
	bool load_sensorinfo(std::string name);
	bool parse_sensorfile(TiXmlDocument doc,std::string name);
	eros::device find_board(uint8_t boardid);
	eros::pin find_pin(const eros::device::ConstPtr& t_board,std::string pinname);
	eros::pin find_pin(const eros::device::ConstPtr& t_device,uint8_t port_id,uint8_t port_pinnumber);

	std::vector<Message> messages;
	std::vector<Board> boards;

	std::vector<Sensor> sensors;
	std::vector<bool> boards_running;
	unsigned char LEDPixelMode;
	uint8_t encoder_count;
	eros::command current_command;
	uint8_t armed_state;

	std::vector<BoardMap> supported_boards;
};
