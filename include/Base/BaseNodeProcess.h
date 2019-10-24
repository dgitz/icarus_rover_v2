// BaseNodeProcess class
//C System Files
#include <sys/time.h>
#include <stdlib.h>
//C++ System Files
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
//ROS Base Functionality
#include "ros/time.h"
//ROS Messages
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include "eros/diagnostic.h"
#include <eros/firmware.h>
#include <eros/heartbeat.h>
#include <eros/device.h>
#include <eros/command.h>
#include <eros/system_state.h>
#include <eros/resource.h>
#include <eros/pin.h>
#include <eros/leverarm.h>
#include <eros/iopins.h>
#include <eros/signal.h>
#include <eros/timesyncinfo.h>
#include <eros/imu.h>
#include <eros/srv_device.h>
#include <eros/srv_connection.h>
#include <eros/srv_leverarm.h>
#include <eros/srv_snapshotstate.h>
#include <eros/srv_sql.h>
#include <eros/srv_pin.h>
#include <eros/subsystem_diagnostic.h>
#include <eros/loadfactor.h>
#include <eros/uptime.h>
#include <eros/tune_controlgroup.h>
#include <eros/view_controlgroup.h>
//Project
#include "../Definitions.h"
#include "../../../eROS/include/DiagnosticClass.h"
#include <nlohmann/json.hpp> //See: https://github.com/nlohmann/json#projects-using-json-for-modern-c

using json = nlohmann::json;
/*! \class BaseNodeProcess BaseNodeProcess.h "BaseNodeProcess.h"
 *  \brief This is a BaseNodeProcess class.  All NodeProcess should be a derived class from this BaseNodeProcess Class. */
class BaseNodeProcess {
public:
	virtual ~BaseNodeProcess()
	{
	}
	//Constants
	//Enums
	//Structs
	//Initialization Functions
	
	/*! \brief Initializes Process.  Should be called right after instantiating variable. */
	void initialize(std::string t_base_node_name,std::string t_node_name,std::string t_hostname,uint8_t t_system,uint8_t t_subsystem,uint8_t t_component)
	{
		unittest_running = false;
		run_time = 0.0;
		base_node_name = t_base_node_name;
		node_name = t_node_name;
		host_name = t_hostname;
		system = t_system;
		subsystem = t_subsystem;
		component = t_component;
		mydevice.DeviceName = t_hostname;
		initialized = false;
		ready = false;
		ready_to_arm = false;
	}
	/*! \brief Enable Diagnostics */
	void enable_diagnostics(std::vector<uint8_t> diagnostic_types)
	{
		root_diagnostic.Node_Name = node_name;
		root_diagnostic.DeviceName = host_name;
		root_diagnostic.System = system;
		root_diagnostic.SubSystem = subsystem;
		root_diagnostic.Component = component;
		std::sort(diagnostic_types.begin(),diagnostic_types.end());
		for(std::size_t i = 0; i < diagnostic_types.size(); ++i)
		{
			eros::diagnostic diag = root_diagnostic;
			diag.Diagnostic_Type = diagnostic_types.at(i);
			if(diagnostic_types.at(i) == SYSTEM_RESOURCE) //This is special, so we don't throw a ton of warn messages when the system launches.
			{
				diag.Level = NOTICE;
				diag.Diagnostic_Message = INITIALIZING;
				diag.Description = "Initializing Resource Monitor.";
			}
			else
			{
				diag.Level = WARN;
				diag.Diagnostic_Message = INITIALIZING;
				diag.Description = "Initializing Diagnostic.";
			}	
			diagnostics.push_back(diag);
		}
	}
	/*! \brief Derived Process Initialization */
	virtual eros::diagnostic finish_initialization() = 0;
	/*! \brief Sets Device Info.  When this function is executed, the Process will now be "initialized". */
	void set_mydevice(eros::device t_device);
	/*! \brief Sets Process Diagnostic info. */
	/*void set_diagnostic(eros::diagnostic t_diagnostic)
	{
		diagnostic = t_diagnostic;
		diagnostic.DeviceName = host_name;
		diagnostic.Node_Name = node_name;
		diagnostic.
	}
	*/
	//Update Functions
	/*! \brief Update function must be implemented in Derived Process.  This is used for all state machine logic, etc. */
	virtual eros::diagnostic update(double t_dt,double t_ros_time) = 0;
	eros::diagnostic update_diagnostic(uint8_t diagnostic_type,uint8_t level,uint8_t message,std::string description);
	eros::diagnostic update_diagnostic(std::string device_name,uint8_t diagnostic_type,uint8_t level,uint8_t message,std::string description);
	eros::diagnostic update_diagnostic(eros::diagnostic diag);

	//Attribute Functions
	eros::device get_mydevice() { return mydevice; }
	double get_runtime() { return run_time; }
	bool is_initialized() { return initialized; }
	bool is_ready() { return ready; }
	bool get_ready_to_arm() { return ready_to_arm; }
	std::vector<eros::diagnostic> get_diagnostics() { return diagnostics; }
	double getROSTime() { return ros_time; }
	uint8_t get_component() { return component; }
	std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }

	//Message Functions
	virtual eros::diagnostic new_devicemsg(const eros::device::ConstPtr& t_device) = 0;
	virtual std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg) = 0;

	//Support Functions
	/*! \brief Must be implemented in Derived Process.  Used for diagnostic testing LEVEL2 and for basic checking of different variables, if they are initialized, etc. */
	virtual std::vector<eros::diagnostic> check_programvariables() = 0;
	/*! \brief Runs Unit Test on Derived Process. */
	std::vector<eros::diagnostic> run_unittest();
	ros::Time convert_time(struct timeval t);
	ros::Time convert_time(double t);
	std::string convert_time_tostr(double t);
	uint8_t convert_signaltype(std::string units,double *conversion_factor);
	std::string exec(const char* cmd,bool wait_for_result);
	//Printing Functions
	void print_message(std::string level,std::string time_str,std::string filename,int line_number,std::string msg);
	void print_diagnostic(eros::diagnostic diag);
protected:
	DiagnosticClass diagnostic_helper;
	std::vector<eros::diagnostic> diagnostics;
	eros::diagnostic update_baseprocess(double t_dt,double t_ros_time);
	eros::device convert_fromptr(const eros::device::ConstPtr& t_ptr);
	eros::pin convert_fromptr(const eros::pin::ConstPtr& t_ptr);
	eros::command convert_fromptr(const eros::command::ConstPtr& t_ptr);
	eros::diagnostic convert_fromptr(const eros::diagnostic::ConstPtr& t_ptr);
	eros::imu convert_fromptr(const eros::imu::ConstPtr& t_ptr);
	eros::signal convert_fromptr(const eros::signal::ConstPtr& t_ptr);
	eros::system_state convert_fromptr(const eros::system_state::ConstPtr& t_ptr);

	bool initialized;
	bool ready;
	eros::device mydevice;
	eros::diagnostic root_diagnostic;

	double run_time,ros_time;

	std::string base_node_name,node_name,host_name;
	uint8_t system,subsystem,component;;
	bool unittest_running;
	bool ready_to_arm;
private:


};
