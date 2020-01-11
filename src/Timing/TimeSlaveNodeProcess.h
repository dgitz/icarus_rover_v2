#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
#include<boost/algorithm/string/split.hpp>
#include<boost/algorithm/string.hpp>
//ROS Base Functionality
//ROS Messages
//Project

/*! \class TimeSlaveNodeProcess TimeSlaveNodeProcess.h "TimeSlaveNodeProcess.h"
 *  \brief This is a TimeSlaveNodeProcess class.  Used for the stimeslave_node node.
 *
 */
class TimeSlaveNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
	struct TimeServer
		{
			uint8_t level;
			std::string name;
			double delay;
			double offset;
			double jitter;
			uint64_t update_count;
			double updated_time;
		};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	void reset()
	{
		ntp_initialized = false;
	}
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	eros::timesyncinfo get_timesyncinfo() { return timesyncinfo; }
	void set_ntp_initialized(bool v){ ntp_initialized = v; } //Unit Testing Only
	void set_unittestingenabled(bool v) { unittesting_enabled = v; } //Unit Testing Only
	void set_exec_result(std::string v) { exec_result = v;} //Unit Testing Only
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);

	//Support Functions
	eros::diagnostic update_timeservers();
	TimeServer get_timeserver(std::string name);
	std::vector<TimeServer> get_timeservers() { return time_servers; }
	void set_timeserver(TimeServer);
	void set_primary_timeserver(std::string t_timeserver) //Unit Testing Only
	{
		primary_time_server = t_timeserver;
		TimeSlaveNodeProcess::TimeServer server;
		server.name = t_timeserver;
		time_servers.push_back(server);
		timesyncinfo.servers.push_back(server.name);
		init_timeservers();
	}

    //Printing Functions
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	void init_timeservers();
	eros::diagnostic update_timeserver(std::string name);
	std::string ros_master_uri;
	std::string primary_time_server;
	bool ntp_initialized;
	bool unittesting_enabled;
	std::string exec_result;
	std::vector<TimeSlaveNodeProcess::TimeServer> time_servers;
	eros::timesyncinfo timesyncinfo;
};
