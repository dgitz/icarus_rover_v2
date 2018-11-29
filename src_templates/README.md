Author: David Gitz
Task: Templates
Nodes:
A. sample_node
Purpose:
Provides a sample node for use creating new nodes.
Instructions:


Implementation:
Main Function should consist of the following:
int main(int argc, char **argv) {
	signal(SIGTERM, signalinterrupt_handler);
	SampleNode *node = new SampleNode();
	bool status = node->initialize(argc,argv);
	while(status)
	{
		status = node->update();
	}
	return 0;
}
Node Initialization should perform the following in order:
    1.	set_basenodename(BASE_NODE_NAME);
	2.  init_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
	3.  init_diagnostic(DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	4.  diagnostic = preinitialize_basenode(argc,argv);
	5.  diagnostic = read_launchparameters();
	6. 	process = new SampleNodeProcess();
	7.  process->init(get_basenodename(),get_nodename());
	8.  process->set_diagnostic(diagnostic);
	9.  diagnostic = finish_initialization();

