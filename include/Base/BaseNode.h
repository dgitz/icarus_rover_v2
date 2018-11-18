// Base class
#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <icarus_rover_v2/diagnostic.h>
#include "icarus_rover_v2/firmware.h"
#include "Definitions.h"
#include "logger.h"
#include "resourcemonitor.h"
#include <std_msgs/Bool.h>
using namespace std;
class BaseNode {
public:
	virtual ~BaseNode()
	{
		printf("%s %s base destroy\n",base_node_name.c_str(),node_name.c_str());
	}
	void set_basenodename(std::string _base_node_name);
	void set_nodename(std::string _node_name)
	{
		node_name = _node_name;
	}
	virtual bool initialize_node() = 0;
	bool initialize_basenode(int argc, char **argv);
	icarus_rover_v2::diagnostic get_baselaunchparameters();
	virtual icarus_rover_v2::diagnostic get_launchparameters() = 0;
	bool update();
	std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }
	double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
	{
		double etime = timer_a.toSec() - timer_b.toSec();
		return etime;
	}
	void set_loop1_rate(double r) { loop1_rate = r; loop1_enabled = true; }
	virtual bool run_loop1() = 0;
	void set_loop2_rate(double r) { loop2_rate = r; loop2_enabled = true; }
	virtual bool run_loop2() = 0;
	void set_loop3_rate(double r) { loop3_rate = r; loop3_enabled = true; }
	virtual bool run_loop3() = 0;
	virtual bool run_01hz() = 0;
	virtual bool run_1hz() = 0;
	virtual bool run_10hz() = 0;



	std::string get_verbositylevel() { return verbosity_level; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	Logger* get_logger() { return logger; }
	boost::shared_ptr<ros::NodeHandle> get_nodehandle() { return n; }


protected:
	boost::shared_ptr<ros::NodeHandle> n;

	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::diagnostic resource_diagnostic;
	icarus_rover_v2::firmware firmware;
	std::string base_node_name;
	std::string node_name;
	char host_name[1024];
private:

	ros::Publisher firmware_pub;
	ros::Publisher resource_pub;
	Logger *logger;
	ResourceMonitor *resourcemonitor;
	double ros_rate;

	ros::Time last_01hz_timer;
	ros::Time last_1hz_timer;
	ros::Time last_10hz_timer;
	bool loop1_enabled;
	ros::Time last_loop1_timer;
	double loop1_rate;

	bool loop2_enabled;
	ros::Time last_loop2_timer;
	double loop2_rate;

	bool loop3_enabled;
	double loop3_rate;
	ros::Time last_loop3_timer;

	std::string verbosity_level;
	ros::Publisher diagnostic_pub;
	icarus_rover_v2::resource resources_used;


};
