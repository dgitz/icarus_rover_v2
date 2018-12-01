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
	//Enums
	//Structs
	struct Sensor
	{
		uint8_t status;
		bool initialized;
		std::string type;
		std::string name;
		std::string remapped_topicname;
		icarus_rover_v2::device connected_hat;
		icarus_rover_v2::pin connected_pin;
		bool convert;
		std::string output_datatype;
		double min_inputvalue;
		double max_inputvalue;
		double min_outputvalue;
		double max_outputvalue;
		icarus_rover_v2::signal signal;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	void set_analyzetiming(bool v) { analyze_timing = v; }
	bool get_analyzetiming() { return analyze_timing; }
	uint8_t get_armedstate() { return armed_state; }

	//Message Functions
	/*! \brief  Process Command Message.
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_armedstatemsg(uint8_t msg);
	icarus_rover_v2::diagnostic new_pinsmsg(const icarus_rover_v2::iopins::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_pinmsg(const icarus_rover_v2::pin::ConstPtr& t_msg);
	icarus_rover_v2::diagnostic new_ppsmsg(std_msgs::Bool t_msg);
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& t_device);
	//Support Functions

	double get_timedelay();

	//Printing Functions

	//Sensor Functions
	Sensor find_sensor(std::string name);
	bool update_sensorinfo(Sensor sensor);
	std::vector<Sensor> get_sensordata() { return sensors; }

	//Generic Hat Functions
	icarus_rover_v2::diagnostic set_hat_running(std::string devicetype,uint16_t id);
	bool is_hat_running(std::string devicetype,uint16_t id);

	//Servo Hat Functions
	std::vector<icarus_rover_v2::pin> get_servohatpins(uint16_t id);
	std::vector<uint16_t> get_servohataddresses();

	//Terminal Hat Functions
	icarus_rover_v2::diagnostic set_terminalhat_initialized();
	std::vector<icarus_rover_v2::pin> get_terminalhatpins(std::string Function,bool match_exact);
	bool set_terminalhatpinvalue(std::string name,int v);

	//GPIO Hat Functions
	bool is_gpiohat_running(uint16_t id);
	std::vector<icarus_rover_v2::pin> get_gpiohatpins(uint16_t id);
	std::vector<uint16_t> get_gpiohataddresses();
	icarus_rover_v2::diagnostic new_message_GetDIOPort1(uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4);
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();

	void init_messages();
	std::string map_PinFunction_ToString(int function);
	double map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output);
	int map_PinFunction_ToInt(std::string Function);
	bool sensors_initialized();
	bool update_sensor(const icarus_rover_v2::device::ConstPtr& t_device,icarus_rover_v2::pin::ConstPtr& t_pin,double tov,double value);
	bool load_sensorinfo(std::string name);
	bool parse_sensorfile(TiXmlDocument doc,std::string name);
	icarus_rover_v2::device find_hat(uint8_t hatid);
	icarus_rover_v2::pin find_pin(const icarus_rover_v2::device::ConstPtr& t_device,std::string pinfunction,uint8_t pinnumber);

	bool hat_present(const icarus_rover_v2::device::ConstPtr& device);

	std::vector<Sensor> sensors;

	std::vector<icarus_rover_v2::device> hats;
	std::vector<bool> hats_running;
	uint64_t pps_counter;
	double time_sincelast_pps;
	bool analyze_timing;
	std::vector<double> timing_diff;
	uint8_t armed_state;
};
