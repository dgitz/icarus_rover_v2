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
#include <icarus_rover_v2/diagnostic.h>
#include "icarus_rover_v2/firmware.h"
#include "icarus_rover_v2/heartbeat.h"
#include <icarus_rover_v2/device.h>
#include "icarus_rover_v2/command.h"
#include <icarus_rover_v2/diagnostic.h>
#include <icarus_rover_v2/device.h>
#include <icarus_rover_v2/resource.h>
#include <icarus_rover_v2/pin.h>
#include <icarus_rover_v2/command.h>
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
#include <icarus_rover_v2/leverarm.h>
#include <icarus_rover_v2/controlgroup.h>
#include <icarus_rover_v2/iopins.h>
#include <icarus_rover_v2/signal.h>
//Project
#include "Definitions.h"

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
	virtual icarus_rover_v2::diagnostic finish_initialization() = 0;
	/*! \brief Sets Device Info.  When this function is executed, the Process will now be "initialized". */
	void set_mydevice(icarus_rover_v2::device t_device);
	/*! \brief Sets Process Diagnostic info. */
	void set_diagnostic(icarus_rover_v2::diagnostic t_diagnostic) { diagnostic = t_diagnostic; }

	//Update Functions
	/*! \brief Update function must be implemented in Derived Process.  This is used for all state machine logic, etc. */
	virtual icarus_rover_v2::diagnostic update(double t_dt,double t_ros_time) = 0;

	//Attribute Functions
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	double get_runtime() { return run_time; }
	bool is_initialized() { return initialized; }
	bool is_ready() { return ready; }
	bool get_ready_to_arm() { return ready_to_arm; }

	//Message Functions
	virtual icarus_rover_v2::diagnostic new_devicemsg(const icarus_rover_v2::device::ConstPtr& device) = 0;
	virtual std::vector<icarus_rover_v2::diagnostic> new_commandmsg(const icarus_rover_v2::command::ConstPtr& t_msg) = 0;

	//Support Functions
	/*! \brief Must be implemented in Derived Process.  Used for diagnostic testing LEVEL2 and for basic checking of different variables, if they are initialized, etc. */
	virtual std::vector<icarus_rover_v2::diagnostic> check_programvariables() = 0;
	/*! \brief Runs Unit Test on Derived Process. */
	std::vector<icarus_rover_v2::diagnostic> run_unittest();
	ros::Time convert_time(struct timeval t);
	ros::Time convert_time(double t);
	//Printing Functions
protected:
	icarus_rover_v2::diagnostic update_baseprocess(double t_dt,double t_ros_time);
	icarus_rover_v2::device convert_fromptr(const icarus_rover_v2::device::ConstPtr& t_ptr);
	icarus_rover_v2::pin convert_fromptr(const icarus_rover_v2::pin::ConstPtr& t_ptr);
	icarus_rover_v2::command convert_fromptr(const icarus_rover_v2::command::ConstPtr& t_ptr);
	icarus_rover_v2::diagnostic convert_fromptr(const icarus_rover_v2::diagnostic::ConstPtr& t_ptr);
	icarus_rover_v2::diagnostic diagnostic;
	bool initialized;
	bool ready;
	icarus_rover_v2::device mydevice;

	double run_time,ros_time;

	std::string base_node_name,node_name,host_name;
	bool unittest_running;
	bool ready_to_arm;
private:


};
