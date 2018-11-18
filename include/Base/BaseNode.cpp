#include "BaseNode.h"
void BaseNode::set_basenodename(std::string _base_node_name)
{
	base_node_name = _base_node_name;
}
bool BaseNode::initialize_basenode(int argc, char **argv)
{
	ros::init(argc,argv,base_node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	host_name[1023] = '\0';
	gethostname(host_name,1023);
	resourcemonitor = new ResourceMonitor(diagnostic,"x86_64",host_name,node_name);
	std::string firmware_topic = "/" + node_name + "/firmware";
	firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,1);
	return true;
}
icarus_rover_v2::diagnostic BaseNode::get_baselaunchparameters()
{
	icarus_rover_v2::diagnostic diag=diagnostic;
	ros_rate = 100.0;
	loop1_enabled = false;
	loop2_enabled = false;
	loop3_enabled = false;
	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing Parameter: verbosity_level. Exiting.";
		diagnostic = diag;
		return diag;
	}
	else
	{
		logger = new Logger(verbosity_level,node_name);
	}


	diagnostic = diag;
	return diag;
}
bool BaseNode::update()
{
	ros::Rate r(ros_rate);
	r.sleep();
	ros::spinOnce();
	double mtime;
	mtime = measure_time_diff(ros::Time::now(),last_01hz_timer);
	if(mtime >= 10.0)
	{
		run_01hz();
		last_01hz_timer = ros::Time::now();
		firmware_pub.publish(firmware);
	}
	mtime = measure_time_diff(ros::Time::now(),last_1hz_timer);
	if(mtime >= 1.0)
	{
		run_1hz();
		last_1hz_timer = ros::Time::now();
		resource_diag = resourcemonitor->update();
		if(resource_diag.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			logger->log_diagnostic(resource_diag);
		}
		else if(resource_diag.Level >= WARN)
		{
			resource_pub.publish(resourcemonitor->get_resourceused());
			logger->log_diagnostic(resource_diag);
		}
		else if(resource_diag.Level <= NOTICE)
		{
			resource_pub.publish(resourcemonitor->get_resourceused());
		}
	}
	mtime = measure_time_diff(ros::Time::now(),last_10hz_timer);
	if(mtime >= 0.1)
	{
		run_10hz();
		last_10hz_timer = ros::Time::now();
	}
	if(loop1_enabled == true)
	{
		mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
		if(mtime >= (1.0/loop1_rate))
		{
			run_loop1();
			last_loop1_timer = ros::Time::now();
		}
	}
	if(loop2_enabled == true)
	{
		mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
		if(mtime >= (1.0/loop2_rate))
		{
			run_loop2();
			last_loop2_timer = ros::Time::now();
		}
	}
	if(loop3_enabled == true)
	{
		mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
		if(mtime >= (1.0/loop3_rate))
		{
			run_loop3();
			last_loop3_timer = ros::Time::now();
		}
	}
	return ros::ok();
}
