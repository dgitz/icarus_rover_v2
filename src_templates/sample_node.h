// Derived class
#include "../include/Base/BaseNode.cpp"
#include "SampleNodeProcess.cpp"
#define SAMPLENODE_MAJOR_RELEASE 3
#define SAMPLENODE_MINOR_RELEASE 0
#define SAMPLENODE_BUILD_NUMBER 0
class SampleNode: public BaseNode {
public:
	~SampleNode()
	{
		printf("node destory\n");
	}
	bool initialize_node()
	{

		diagnostic.DeviceName = host_name;
		diagnostic.Node_Name = node_name;
		diagnostic.System = ROVER;
		diagnostic.SubSystem = ROBOT_CONTROLLER;
		diagnostic.Component = TIMING_NODE;

		diagnostic.Diagnostic_Type = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = INITIALIZING;
		diagnostic.Description = "Node Initializing";

		process = new SampleNodeProcess;

		firmware.Generic_Node_Name = get_basenodename();
		firmware.Node_Name = node_name;
		firmware.Description = "Latest Rev: 17-November-2018";
		firmware.Major_Release = SAMPLENODE_MAJOR_RELEASE;
		firmware.Minor_Release = SAMPLENODE_MINOR_RELEASE;
		firmware.Build_Number = SAMPLENODE_BUILD_NUMBER;
		return true;
	}
	icarus_rover_v2::diagnostic get_launchparameters();
	void print_runtime();
	bool run_01hz();
	bool run_1hz();
	bool run_10hz();
	bool run_loop1();
	bool run_loop2();
	bool run_loop3();

	static void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg);
private:
	std::string base_node_name;
	ros::Subscriber pps1_sub;
	SampleNodeProcess *process;

};
