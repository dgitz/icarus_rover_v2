#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>

/*! \class HatControllerNodeProcess HatControllerNodeProcess.h "HatControllerNodeProcess.h"
 *  \brief This is a HatControllerNodeProcess class.  Used for the hatcontroller_node node.
 *
 */
class HatControllerNodeProcess: public BaseNodeProcess {
public:
	//Constants
	const uint8_t TIMING_BUFFER_LENGTH = 100;
	static const uint16_t SUPPORTED_HATCOUNT = 3;  //Number of Different Hats supported, by PN
	//Enums
	//Structs
	struct Sensor
	{
		uint8_t status;
		bool initialized;
		double conversion_factor;
		std::string type;
		std::string name;
		std::string remapped_topicname;
		eros::device connected_hat;
		eros::pin connected_pin;
		bool convert;
		std::string output_datatype;
		double min_inputvalue;
		double max_inputvalue;
		double min_outputvalue;
		double max_outputvalue;
		eros::signal signal;
	};
	struct PinDefinition
	{
		std::string PinName;
		uint8_t port_id;
		uint8_t port_pinnumber;
	};
	struct HatMap
	{
		std::string DeviceType;
		std::string FAST_PN;
		std::vector<PinDefinition> PinMap;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	bool initialize_supportedhats();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_analyzetiming(bool v) { analyze_timing = v; }
	bool get_analyzetiming() { return analyze_timing; }
	uint8_t get_armedstate() { return armed_state; }

	//Message Functions
	/*! \brief  Process Command Message.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_armedstatemsg(uint8_t msg);
	eros::diagnostic new_pinmsg(const eros::pin::ConstPtr& t_msg);
	eros::diagnostic new_ppsmsg(std_msgs::Bool t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& t_device);
	//Support Functions
	eros::pin find_pin(const eros::device::ConstPtr& t_device,uint8_t port_id,uint8_t port_pinnumber);
	eros::pin find_pin(const eros::device::ConstPtr& hat,std::string pin_name);
	eros::diagnostic update_pin(std::string device_type,uint8_t device_id,std::string pin_name,eros::pin new_pin);
	double get_timedelay();

	//Printing Functions
	//Sensor Functions
	Sensor find_sensor(std::string name);
	bool update_sensorinfo(Sensor sensor);
	std::vector<Sensor> get_sensordata() { return sensors; }

	//Generic Hat Functions
	eros::diagnostic new_message_GetDIOPort1(std::string device_type,uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4);
	eros::diagnostic new_message_GetANAPort1(std::string device_type,uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4,uint16_t v5, uint16_t v6);
	eros::diagnostic new_message_GetANAPort2(std::string device_type,uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4,uint16_t v5, uint16_t v6);
	std::vector<HatMap> get_allsupportedhats() { return supported_hats; }
	HatMap get_hatmap_bypartnumber(std::string partnumber);
	eros::diagnostic set_hat_running(std::string devicetype,uint16_t id);
	bool is_hat_running(std::string devicetype,uint16_t id);
	std::vector<eros::device> get_hats() { return hats; }
	std::vector<eros::pin> get_pins_byport(std::string devicetype,uint16_t id,uint8_t port_id);

	//Servo Hat Functions
	std::vector<eros::pin> get_servohatpins(uint16_t id);
	std::vector<uint16_t> get_servohataddresses();

	//Terminal Hat Functions
	eros::diagnostic set_terminalhat_initialized();
	std::vector<eros::pin> get_terminalhatpins();
	std::vector<eros::pin> get_terminalhatpins(std::string Function,bool match_exact);
	bool set_terminalhatpinvalue(std::string name,int v);

	//GPIO Hat Functions
	bool is_gpiohat_running(uint16_t id);
	std::vector<eros::pin> get_gpiohatpins(uint16_t id);
	std::vector<uint16_t> get_gpiohataddresses();

protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
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
	bool update_sensor(const eros::device::ConstPtr& t_device,eros::pin::ConstPtr& t_pin,double tov,double value);
	bool load_sensorinfo(std::string name);
	bool parse_sensorfile(TiXmlDocument doc,std::string name);
	eros::device find_hat(uint8_t hatid);
	
	bool hat_present(const eros::device::ConstPtr& device);
	std::vector<HatMap> supported_hats;
	std::vector<Sensor> sensors;

	std::vector<eros::device> hats;
	std::vector<bool> hats_running;
	uint64_t pps_counter;
	double time_sincelast_pps;
	bool analyze_timing;
	std::vector<double> timing_diff;
	uint8_t armed_state;
};
