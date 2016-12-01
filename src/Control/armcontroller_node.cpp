#include "armcontroller_node.h"
//Start User Code: Firmware Definition
#define ARMCONTROLLERNODE_MAJOR_RELEASE 1
#define ARMCONTROLLERNODE_MINOR_RELEASE 1
#define ARMCONTROLLERNODE_BUILD_NUMBER 1
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_slowrate_code()
{
	sleep(5);
	ROS_INFO("Starting");
	move_group_interface::MoveGroup group("robot_arm");
	ROS_INFO("Group Instantiated");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ROS_INFO("Planning Scene INterface created");
	group.setEndEffectorLink("leftarm_gripper");
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0;
	target_pose1.position.y = 0;
	target_pose1.position.z = 0;
	group.setRandomTarget();
	geometry_msgs::PoseStamped targetpose = group.getPoseTarget();
	ROS_INFO("Target Pose x: %f y: %f z: %f",targetpose.pose.position.x,targetpose.pose.position.y,targetpose.pose.position.z);

	//group.setPoseTarget(target_pose1);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	ROS_INFO("Starting Path Plan.");

	group.setPlanningTime(2.0);
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	display_trajectory.trajectory_start = my_plan.start_state_;
	display_trajectory.trajectory.push_back(my_plan.trajectory_);
	display_publisher.publish(display_trajectory);
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
	}
	//logger->log_debug("Running slow rate code.");

	return true;
}
bool run_veryslowrate_code()
{

	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "tfhelper_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 22-Oct-2016";
	fw.Major_Release = ARMCONTROLLERNODE_MAJOR_RELEASE;
	fw.Minor_Release = ARMCONTROLLERNODE_MINOR_RELEASE;
	fw.Build_Number = ARMCONTROLLERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
//End User Code: Functions

//Start Template Code: Functions
int main(int argc, char **argv)
{
	node_name = "armcontroller_node";
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
    //Start Template Code: Initialization and Parameters
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
	diagnostic_status.Component = CONTROLLER_NODE;

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
    display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	ROS_INFO("Publisher Created");
    //Arm command topic: joint_sub=simExtRosInterface_subscribe('/dgitsrosmaster_armcontroller_node/leftarm_command','sensor_msgs/Joy','setArmJointMotor_cb')

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
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
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
		device_initialized = true;
	}
}
//End Template Code: Functions
