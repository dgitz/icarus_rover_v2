// BaseNodeProcess class
//C System Files
#include <sys/time.h>
//C++ System Files
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
//ROS Base Functionality
#include "ros/time.h"
//ROS Messages
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/JointState.h>
#include <eros/diagnostic.h>
#include <eros/firmware.h>
#include <eros/heartbeat.h>
#include <eros/device.h>
#include <eros/command.h>
#include <eros/resource.h>
#include <eros/pin.h>
#include <eros/leverarm.h>
#include <eros/controlgroup.h>
#include <eros/iopins.h>
#include <eros/signal.h>
#include <eros/timesyncinfo.h>
#include <eros/imu.h>
#include <eros/srv_device.h>
#include <eros/srv_connection.h>
#include <eros/srv_leverarm.h>
//Project
#include "../Definitions.h"

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
	void initialize(std::string t_base_node_name,std::string t_node_name,std::string t_hostname)
	{
		unittest_running = false;
		run_time = 0.0;
		base_node_name = t_base_node_name;
		node_name = t_node_name;
		host_name = t_hostname;
		mydevice.DeviceName = t_hostname;
		initialized = false;
		ready = false;
		ready_to_arm = false;
	}
	/*! \brief Derived Process Initialization */
	virtual eros::diagnostic finish_initialization() = 0;
	/*! \brief Sets Device Info.  When this function is executed, the Process will now be "initialized". */
	void set_mydevice(eros::device t_device);
	/*! \brief Sets Process Diagnostic info. */
	void set_diagnostic(eros::diagnostic t_diagnostic)
	{
		diagnostic = t_diagnostic;
		diagnostic.DeviceName = host_name;
		diagnostic.Node_Name = node_name;
	}

	//Update Functions
	/*! \brief Update function must be implemented in Derived Process.  This is used for all state machine logic, etc. */
	virtual eros::diagnostic update(double t_dt,double t_ros_time) = 0;

	//Attribute Functions
	eros::device get_mydevice() { return mydevice; }
	eros::diagnostic get_diagnostic() { return diagnostic; }
	double get_runtime() { return run_time; }
	bool is_initialized() { return initialized; }
	bool is_ready() { return ready; }
	bool get_ready_to_arm() { return ready_to_arm; }

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
	//Printing Functions
protected:
	eros::diagnostic update_baseprocess(double t_dt,double t_ros_time);
	eros::device convert_fromptr(const eros::device::ConstPtr& t_ptr);
	eros::pin convert_fromptr(const eros::pin::ConstPtr& t_ptr);
	eros::command convert_fromptr(const eros::command::ConstPtr& t_ptr);
	eros::diagnostic convert_fromptr(const eros::diagnostic::ConstPtr& t_ptr);
	eros::imu convert_fromptr(const eros::imu::ConstPtr& t_ptr);
	eros::diagnostic diagnostic;
	bool initialized;
	bool ready;
	eros::device mydevice;

	double run_time,ros_time;

	std::string base_node_name,node_name,host_name;
	bool unittest_running;
	bool ready_to_arm;
private:


};
