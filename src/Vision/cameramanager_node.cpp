#include "cameramanager_node.h"
//Start Template Code: Firmware Definition
#define CAMERAMANAGERNODE_MAJOR_RELEASE 1
#define CAMERAMANAGERNODE_MINOR_RELEASE 2
#define CAMERAMANAGERNODE_BUILD_NUMBER 0
//End Template Code: Firmware Definition
//Start User Code: Functions
icarus_rover_v2::diagnostic publish_tf_frames(icarus_rover_v2::diagnostic diag)
{

	static tf::TransformBroadcaster br;

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
	diag.Diagnostic_Type = SOFTWARE;
	diag.Diagnostic_Message = NOERROR;
	diag.Level = INFO;
	diag.Description = "Broadcasting tf frames.";
	return diag;
}
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	diagnostic_status = publish_tf_frames(diagnostic_status);
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
	fw.Description = "Latest Rev: 27-Nov-2016";
	fw.Major_Release = CAMERAMANAGERNODE_MAJOR_RELEASE;
	fw.Minor_Release = CAMERAMANAGERNODE_MINOR_RELEASE;
	fw.Build_Number = CAMERAMANAGERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}

//End User Code: Functions

//Start Initialize Function


//Start Main Loop
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
        return 0; 
    }
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok())
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
//End Main Loop
bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization, Parameters and Topics
    printf("Node name: %s\r\n",node_name.c_str());
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(nh.getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("FATAL",ros::this_node::getName());
		logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}
	std::string param_loop_rate = node_name +"/loop_rate";
	if(nh.getParam(param_loop_rate,rate) == false)
	{
		logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
		return false;
	}
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;
	std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
	//End Template Code: Initialization, Parameters and Topics

	//Start User Code: Initialization, Parameters and Topics
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

	bool search_for_topics = true;
	int topicindex = 1;
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
			add_new_topic = true;
			search_for_topics = true;
		}
		if(add_new_topic == true)
		{
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
			topicindex++;
		}
	}




	//End User Code: Initialization, Parameters and Topics
    logger->log_info("Initialized!");
    return true;
}
//End Initialize Function

//Start Template Code: Functions
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
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
//End Template Code: Functions
