OBSOLETE!!!
#include "cameramanager_node.h"
//Start User Code: Firmware Definition
#define CAMERAMANAGERNODE_MAJOR_RELEASE 1
#define CAMERAMANAGERNODE_MINOR_RELEASE 2
#define CAMERAMANAGERNODE_BUILD_NUMBER 3
//End User Code: Firmware Definition
//Start User Code: Functions
icarus_rover_v2::diagnostic publish_tf_frames(icarus_rover_v2::diagnostic diag)
{
	/*static tf::TransformBroadcaster br;

	tf::Transform transform_mapodom;
	transform_mapodom.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q_mapodom;
	q_mapodom.setRPY(0,0,0);
	transform_mapodom.setRotation(q_mapodom);
	br.sendTransform(tf::StampedTransform(transform_mapodom, ros::Time::now(), "map", "odom"));


	tf::Transform transform_cam1;
	transform_cam1.setOrigin(tf::Vector3(0.0,0.0,0.0));
	tf::Quaternion q_cam1;
	q_cam1.setRPY(0,0,0);
	transform_cam1.setRotation(q_cam1);
	br.sendTransform(tf::StampedTransform(transform_cam1, ros::Time::now(), "base_link", "asusxtioncamera_link"));
	*/
	diag.Diagnostic_Type = SOFTWARE;
	diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	diag.Level = ERROR;
	diag.Description = "Trying to broadcast tf frames but not enabled.";
	return diag;
}
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	//diagnostic_status = publish_tf_frames(diagnostic_status);
	diagnostic_pub.publish(diagnostic_status);
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	for(int i = 0; i < external_tasks.size();i++)
	{
		icarus_rover_v2::heartbeat newbeat;
		newbeat.stamp = ros::Time::now();
		newbeat.Node_Name = external_tasks.at(i).TaskName;
		external_tasks.at(i).heartbeat_pub.publish(newbeat);
	}
	//logger->log_debug("Running medium rate code.");
	return true;
}
bool run_slowrate_code()
{
	if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic;
		resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_warn("Couldn't read resources used.");
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
		for(int i = 0; i < external_tasks.size();i++)
		{
			external_tasks.at(i).diagnostic = external_tasks.at(i).resourcemonitor.update();
			if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
			{
				external_tasks.at(i).diagnostic_pub.publish(external_tasks.at(i).diagnostic);
				//diagnostic_pub.publish(resource_diagnostic);
				logger->log_warn(resource_diagnostic.Description.c_str());
				char tempstr[255];
				sprintf(tempstr,"Couldn't read resources used for external task: %s",external_tasks.at(i).TaskName.c_str());
				logger->log_warn(tempstr);

			}
			else if(resource_diagnostic.Level >= WARN)
			{
				external_tasks.at(i).resources_used = external_tasks.at(i).resourcemonitor.get_resourceused();
				external_tasks.at(i).resource_pub.publish(external_tasks.at(i).resources_used);
				external_tasks.at(i).diagnostic_pub.publish(external_tasks.at(i).diagnostic);
			}
			else if(resource_diagnostic.Level <= NOTICE)
			{

				external_tasks.at(i).resources_used = external_tasks.at(i).resourcemonitor.get_resourceused();
				external_tasks.at(i).resource_pub.publish(external_tasks.at(i).resources_used);
				external_tasks.at(i).diagnostic_pub.publish(external_tasks.at(i).diagnostic);
			}
		}
	}
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "cameramanager_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 4-Dec-2016";
	fw.Major_Release = CAMERAMANAGERNODE_MAJOR_RELEASE;
	fw.Minor_Release = CAMERAMANAGERNODE_MINOR_RELEASE;
	fw.Build_Number = CAMERAMANAGERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}

std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");

	if(status == true)
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}

void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	//logger->log_info("Got command");
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
			std::vector<icarus_rover_v2::diagnostic> diaglist = check_program_variables();
			for(int i = 0; i < diaglist.size();i++) { diagnostic_pub.publish(diaglist.at(i)); }
		}
		else if(msg->Option1 == LEVEL3)
		{
		}
		else if(msg->Option1 == LEVEL4)
		{
		}
		else
		{
			logger->log_error("Shouldn't get here!!!");
		}
	}
}
//End User Code: Functions

int main(int argc, char **argv)
{
	node_name = "cameracapture_node";
    ros::init(argc, argv, node_name);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    
    if(initialize(n) == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
		kill_node = 1;
    }
    ros::Rate loop_rate(rate);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .02)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
				run_mediumrate_code();
				medium_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,slow_timer);
			if(mtime > 1.0)
			{
				run_slowrate_code();
				slow_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,veryslow_timer);
			if(mtime > 10.0)
			{
				run_veryslowrate_code();
				veryslow_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
    logger->log_notice("Node Finished Safely.");
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = VISION_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

    std::string param_verbosity_level = node_name +"/verbosity_level";
    if(nh.getParam(param_verbosity_level,verbosity_level) == false)
    {
        logger = new Logger("WARN",ros::this_node::getName());
        logger->log_warn("Missing Parameter: verbosity_level");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    std::string param_loop_rate = node_name +"/loop_rate";
    if(nh.getParam(param_loop_rate,rate) == false)
    {
        logger->log_warn("Missing Parameter: loop_rate.");
        return false;
    }
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);

    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    //End Template Code: Initialization and Parameters
	
    //Start User Code: Initialization and Parameters
	bool search_for_topics = true;
	int topicindex = 1;
	ros::V_string nodelist;
	ros::master::getNodes(nodelist);
	while(search_for_topics == true)
	{
		std::string taskname;
		bool add_new_topic = false;
		std::string param_task = node_name +"/task_monitor" + boost::lexical_cast<std::string>(topicindex);
		if(nh.getParam(param_task,taskname) == false)
		{
			char tempstr[255];
			sprintf(tempstr,"Didn't find %s Not adding anymore.",param_task.c_str());
			logger->log_info(tempstr);
			add_new_topic = false;
			search_for_topics = false;
		}
		else
		{
			//boost::replace_all(taskname, "/", "-");
			printf("Found: %s\n",taskname.c_str());
			add_new_topic = true;
			search_for_topics = true;
		}
		if(add_new_topic == true)
		{
			std::size_t asterick_found = taskname.find("*");
			if (asterick_found==std::string::npos)
			{
				printf("Task: %s\n",taskname.c_str());

				CameraTask newcameratask;
				newcameratask.TaskName = taskname;
				std::string topic_cameratask_resource = "/" + newcameratask.TaskName + "/resource";
				ros::Publisher r_pub = nh.advertise<icarus_rover_v2::resource>(topic_cameratask_resource,1000);
				newcameratask.resource_pub = r_pub;
				std::string topic_cameratask_diagnostic = "/" + newcameratask.TaskName + "/diagnostic";
				ros::Publisher d_pub = nh.advertise<icarus_rover_v2::diagnostic>(topic_cameratask_diagnostic,1000);
				std::string topic_cameratask_heartbeat = "/" + newcameratask.TaskName + "/heartbeat";
				ros::Publisher h_pub = nh.advertise<icarus_rover_v2::heartbeat>(topic_cameratask_heartbeat,1000);
				newcameratask.diagnostic_pub = d_pub;
				newcameratask.heartbeat_pub = h_pub;
				newcameratask.diagnostic = diagnostic_status;
				newcameratask.diagnostic.Node_Name = newcameratask.TaskName;
				newcameratask.diagnostic.Description = "External Task Initializing";
				external_tasks.push_back(newcameratask);
			}
			else
			{

				std::string start = taskname.substr(0,asterick_found);
				std::string end = taskname.substr(asterick_found,taskname.length());

				if(end.length() > 1) //This should only be an "*" for now
				{
					char tempstr[255];
					sprintf(tempstr,"Task: %s with wildcard not supported.  Wildcard must be at end of name.  Exiting.",taskname.c_str());
					logger->log_error(tempstr);
					kill_node = 1;
					return false;
				}
				std::vector<std::string> extratasks;
				for(int i = 0; i < nodelist.size();i++)
				{
					std::size_t foundtask = nodelist.at(i).find(start);
					if(foundtask != std::string::npos)
					{
						CameraTask newcameratask;
						newcameratask.TaskName = nodelist.at(i);
						std::string topic_cameratask_resource = "/" + newcameratask.TaskName + "/resource";
						ros::Publisher r_pub = nh.advertise<icarus_rover_v2::resource>(topic_cameratask_resource,1000);
						newcameratask.resource_pub = r_pub;
						std::string topic_cameratask_diagnostic = "/" + newcameratask.TaskName + "/diagnostic";
						ros::Publisher d_pub = nh.advertise<icarus_rover_v2::diagnostic>(topic_cameratask_diagnostic,1000);
						std::string topic_cameratask_heartbeat = "/" + newcameratask.TaskName + "/heartbeat";
						ros::Publisher h_pub = nh.advertise<icarus_rover_v2::heartbeat>(topic_cameratask_heartbeat,1000);
						newcameratask.diagnostic_pub = d_pub;
						newcameratask.heartbeat_pub = h_pub;
						newcameratask.diagnostic = diagnostic_status;
						newcameratask.diagnostic.Node_Name = newcameratask.TaskName;
						newcameratask.diagnostic.Description = "External Task Initializing";
						external_tasks.push_back(newcameratask);
					}
				}
			}
			topicindex++;
		}
	}
    //Finish User Code: Initialization and Parameters

    //Start Template Code: Final Initialization.
	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Initialized";
	diagnostic_pub.publish(diagnostic_status);
    logger->log_info("Initialized!");
    return true;
    //End Template Code: Finish Initialization.
}
//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;

	if((newdevice.DeviceName == hostname) && (device_initialized == false))
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		for(int i = 0; i < external_tasks.size();i++)
		{
			external_tasks.at(i).resourcemonitor.init(external_tasks.at(i).diagnostic,myDevice.Architecture,myDevice.DeviceName,external_tasks.at(i).TaskName);
		}
		device_initialized = true;
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
