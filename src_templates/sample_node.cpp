#include "sample_node.h"
icarus_rover_v2::diagnostic SampleNode::get_launchparameters()
{
	process->set_diagnostic(diagnostic);
	process->init(base_node_name,node_name);
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",5,PPS1_Callback);

	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
bool SampleNode::run_01hz()
{
	return true;
}
bool SampleNode::run_1hz()
{
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	process->set_diagnostic(diag);
	process->print_runtime();
	get_logger()->log_diagnostic(diag);
	return true;
}
bool SampleNode::run_10hz()
{
	process->update(0.1);
	return true;
}
bool SampleNode::run_loop1()
{
	return true;
}
bool SampleNode::run_loop2()
{
	return true;
}
bool SampleNode::run_loop3()
{
	return true;
}

void SampleNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
}

int main(int argc, char **argv) {
	SampleNode *node = new SampleNode();

	node->set_basenodename("sample_node");
	bool init_status = node->initialize_basenode(argc,argv);
	init_status = node->initialize_node();
	icarus_rover_v2::diagnostic diag = node->get_baselaunchparameters();
	INIT RESOURCE MONITOR?
	if(diag.Level >= WARN)
	{
		printf("Error: %s\n",diag.Description.c_str());
		return 0;
	}
	diag = node->get_launchparameters();
	bool status = true;
	node->set_loop1_rate(1.0);
	node->set_loop2_rate(2.0);
	node->set_loop3_rate(4.0);
	while(status)
	{
		status = node->update();
		//node->print_runtime();
	}

	return 0;
}
