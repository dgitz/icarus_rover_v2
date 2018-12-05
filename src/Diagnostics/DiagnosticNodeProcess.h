#include "../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project

/*! \class DiagnosticNodeProcess DiagnosticNodeProcess.h "DiagnosticNodeProcess.h"
 *  \brief This is a DiagnosticNodeProcess class.  Used for the diagnostic_node node.
 *
 */
class DiagnosticNodeProcess: public BaseNodeProcess {
public:
	//Constants
	const double WORSTDIAG_TIMELIMIT = 5.0f;
	const double WAITNODE_BRINGUP_TIME = 20.0f;
	//Enums

	//Structs
	struct Task
	{
		std::string Task_Name;
		double last_diagnostic_received;
		double last_resource_received;
		double last_heartbeat_received;
		int16_t PID;
		int16_t CPU_Perc;
		int64_t RAM_MB;
		uint8_t last_diagnostic_level;
		std::string resource_topic;
		std::string diagnostic_topic;
		std::string heartbeat_topic;
	};

	struct DeviceResourceAvailable
	{
		std::string Device_Name;
		int16_t CPU_Perc_Available;
		int64_t RAM_Mb_Available;
	};
	struct DiagLevel
	{
		uint8_t Level;
		double last_time;
		icarus_rover_v2::diagnostic diag;
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
    /*! \brief Checks tasks for CPU,RAM issues and for heartbeats */
	std::vector<icarus_rover_v2::diagnostic> check_tasks();
	//Attribute Functions
	bool get_RobotUnderRemoteControl() { return RCControl; }
	double get_worstdiag_timelimit() { return WORSTDIAG_TIMELIMIT; }
	double get_waitnode_bringup_time() { return WAITNODE_BRINGUP_TIME; }
	uint8_t get_lcdwidth(){ return lcd_width; }
	uint8_t get_lcdheight() { return lcd_height; }
	void set_batterylevel(double v) { battery_level = v;}
	void set_batteryvoltage(double v) { voltage_received = true; battery_voltage = v;}
    /*! \brief Disable LCD */
	void no_connectedlcd()
	{
		lcd_available = false;
	}
	bool get_lcdavailable() { return lcd_available; }
	void set_resourcethresholds(int RAM_usage_threshold_MB_,int CPU_usage_threshold_percent_)
	{
		RAM_usage_threshold_MB = RAM_usage_threshold_MB_;
		CPU_usage_threshold_percent = CPU_usage_threshold_percent_;
	}
	std::vector<Task> get_TaskList() { return TaskList; }
	std::vector<DeviceResourceAvailable> get_DeviceResourceAvailableList() { return DeviceResourceAvailableList; }
	void add_Task(Task v);
	void set_log_resources_used(bool v) { log_resource_used = v; }
	bool get_log_resources_used() { return log_resource_used; }
	//Message Functions
	/*! \brief  Process Command Message. */
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg);
    /*! \brief  Process Device Message. */
	icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device);
    /*! \brief  Process Heartbeat Message. */
	void new_heartbeatmsg(std::string topicname);
    /*! \brief  Process Resource Message. */
	void new_resourcemsg(std::string topicname,const icarus_rover_v2::resource::ConstPtr& resource);
    /*! \brief  Process Diagnostic Message. */
	void new_diagnosticmsg(std::string topicname,const icarus_rover_v2::diagnostic::ConstPtr& diagnostic);
    /*! \brief  Process Armed State Message. */
	void new_armedstatemsg(uint8_t v) { armed_state = v; }
	//Support Functions
	std::string build_lcdmessage();
	//Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<icarus_rover_v2::diagnostic> check_programvariables();
	std::string get_batterylevelstr(double v);
	unsigned char get_lcdclockchar(int v);
	std::string get_batteryvoltagestr();
	std::string get_armedstatestr(uint8_t v);
	std::string get_diagstr();
	std::string get_lcdcommandstr();
	void init_diaglevels();


	std::vector<Task> TaskList;
	std::vector<DeviceResourceAvailable> DeviceResourceAvailableList;
	int RAM_usage_threshold_MB;
	int CPU_usage_threshold_percent;

	double last_cmddiagnostic_timer;
	double last_cmd_timer;
	double battery_level;
	uint8_t lcd_width;
	uint8_t lcd_height;
	std::string lcd_partnumber;
	int lcd_clock;
	bool voltage_received;
	double battery_voltage;
	std::vector<DiagLevel> diaglevels;
	bool bad_diagnostic_received;
	bool any_diagnostic_received;
	bool lcd_available;
	icarus_rover_v2::command current_command;
	bool command_received;
	double lcdclock_timer;
	uint8_t armed_state;
	bool log_resource_used;
	bool RCControl;
};
