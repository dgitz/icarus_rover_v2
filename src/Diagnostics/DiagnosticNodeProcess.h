#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
#include <eros/battery.h>
//Project
#include "../../../eROS/include/DiagnosticClass.h"
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

	struct SubSystemDiagnostic
	{
		uint8_t Diagnostic_Type;
		uint8_t Level;
		std::vector<eros::diagnostic> diagnostics;
		std::vector<unsigned long> level_counters;
	};
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
		eros::diagnostic diag;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);
    /*! \brief Checks tasks for CPU,RAM issues and for heartbeats */
	std::vector<eros::diagnostic> check_tasks();
	//Attribute Functions
	bool get_RobotUnderRemoteControl() { return RCControl; }
	double get_worstdiag_timelimit() { return WORSTDIAG_TIMELIMIT; }
	double get_waitnode_bringup_time() { return WAITNODE_BRINGUP_TIME; }
	uint8_t get_lcdwidth(){ return lcd_width; }
	uint8_t get_lcdheight() { return lcd_height; }
	void set_batterylevel(double v) { battery_level = v;}
	void set_batteryvoltage(double v) { voltage_received = true; battery_voltage = v;}
	std::vector<SubSystemDiagnostic> get_subsystem_diagnostics() { return subsystem_diagnostics; }
	SubSystemDiagnostic get_subsystem_diagnostic(uint8_t Diagnostic_Type);
	/*! \brief Get Subsystem Diagnostic Message to Publish .*/
	eros::subsystem_diagnostic get_eros_subsystem_diagnostic();
    /*! \brief Disable LCD */
	void no_connectedlcd()
	{
		update_diagnostic(REMOTE_CONTROL,INFO,NOERROR,"LCD is Disabled.");
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
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
    /*! \brief  Process Device Message. */
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
    /*! \brief  Process Heartbeat Message. */
	void new_heartbeatmsg(std::string topicname);
    /*! \brief  Process Resource Message. */
	void new_resourcemsg(std::string topicname,const eros::resource::ConstPtr& resource);
    /*! \brief  Process Diagnostic Message. */
	void new_diagnosticmsg(std::string topicname,const eros::diagnostic::ConstPtr& diagnostic);
    /*! \brief  Process Armed State Message. */
	void new_armedstatemsg(uint8_t v) { armed_state = v; }
	uint8_t get_armedstate() { return armed_state; }
	
	//Support Functions
	std::string build_lcdmessage();
	DiagnosticClass diagnostic_helper;
	//Printing Functions
	std::string print_subsystem_diagnostics();
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	std::string get_batterylevelstr(double v);
	unsigned char get_lcdclockchar(int v);
	std::string get_batteryvoltagestr();
	std::string get_armedstatestr(uint8_t v);
	std::string get_diagstr();
	std::string get_lcdcommandstr();
	void init_diaglevels();
	void init_subsystemdiagnostics();


	std::vector<Task> TaskList;
	std::vector<DeviceResourceAvailable> DeviceResourceAvailableList;
	std::vector<SubSystemDiagnostic> subsystem_diagnostics;
	eros::subsystem_diagnostic eros_subsystem_diagnostic;
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
	eros::command current_command;
	bool command_received;
	double lcdclock_timer;
	uint8_t armed_state;
	bool log_resource_used;
	bool RCControl;
};
