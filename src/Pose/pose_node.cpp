#include "pose_node.h"
//Start User Code: Firmware Definition
#define POSENODE_MAJOR_RELEASE 0
#define POSENODE_MINOR_RELEASE 0
#define POSENODE_BUILD_NUMBER 2
//End User Code: Firmware Definition
//Start User Code: Functions
void simpose_Callback(const icarus_rover_v2::pose::ConstPtr& msg)
{
	icarus_rover_v2::pose p;
	p.header.stamp = msg->header.stamp;
	p.north.value = msg->north.value;
	p.east.value = msg->east.value;
	p.yawrate.value = msg->yawrate.value;
	p.yaw.value = msg->yaw.value;
	p.elev.value = msg->elev.value;
	p.wheelspeed.value = msg->wheelspeed.value;
	p.groundspeed.value = msg->groundspeed.value;
	process->set_simpose(p);
}
void attitude_Callback(const roscopter::Attitude::ConstPtr& msg)
{
    logger->log_diagnostic(process->new_kalmanfilter_signal("Yaw",1,msg->yaw));
    logger->log_diagnostic(process->new_kalmanfilter_signal("Yawrate",0,msg->yawspeed));
}
void throttle_command_Callback(const std_msgs::Float32::ConstPtr& msg)
{
    process->set_throttlecommand(msg->data);
}
void steer_command_Callback(const std_msgs::Float32::ConstPtr& msg)
{
	process->set_steercommand(msg->data);
}
void encoder_Callback(const icarus_rover_v2::encoder::ConstPtr& msg)
{
	process->set_encoder(msg->data[0],msg->data[1]);
}
void gps_Callback(const icarus_rover_v2::pose::ConstPtr& msg)
{
	icarus_rover_v2::pose gps;
	gps.north.value = msg->north.value;
	gps.east.value = msg->east.value;
	process->set_gps(gps);
}
bool run_loop1_code()
{
    logger->log_diagnostic(process->update(measure_time_diff(ros::Time::now(),last_loop1_timer)));
	return true;
}
bool run_loop2_code()
{
    if(process->is_poseready() == true)
    {
        icarus_rover_v2::pose pose = process->get_pose();
        pose.header.stamp = ros::Time::now();
        pose_pub.publish(pose);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pose.east.value,pose.north.value, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, pose.yaw.value);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        //sensor_msgs::JointState joints;
        //joints.header.stamp = ros::Time::now();
        //joints.name.resize(1);
        //joints.name[0] = "joint_camera_tilt";
        //joints.name[1] = "joint_camera_tilt";
        //joints.position.resize(1);
        //joints.position[0] = tilt_angle;
        //joints.position[1] = tilt_angle;
        //jointstate_pub.publish(joints);
        //pan_angle = pan_angle + M_PI/100.0;
        //if(pan_angle > M_PI/4.0) { pan_angle = -M_PI/4.0; }
        //tilt_angle = tilt_angle + M_PI/100.0;
        //        if(tilt_angle > M_PI/4.0) { tilt_angle = -M_PI/4.0; }
    }
 	return true;
}
bool run_loop3_code()
{
 	return true;
}
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "pose_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 12-Oct-2017";
	fw.Major_Release = POSENODE_MAJOR_RELEASE;
	fw.Minor_Release = POSENODE_MINOR_RELEASE;
	fw.Build_Number = POSENODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
    if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
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
bool run_10Hz_code()
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

    if(diagnostic_status.Level > NOTICE)
    {
        diagnostic_pub.publish(diagnostic_status);
    }
    return true;
}
int main(int argc, char **argv)
{
	node_name = "pose_node";
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
    ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
    last_10Hz_timer = ros::Time::now();
    double mtime;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
            if(run_loop1 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
                if(mtime >= (1.0/loop1_rate))
                {
                    run_loop1_code();
                    last_loop1_timer = ros::Time::now();
                }
            }
            if(run_loop2 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
                if(mtime >= (1.0/loop2_rate))
                {
                    run_loop2_code();
                    last_loop2_timer = ros::Time::now();
                }
            }
            if(run_loop3 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
                if(mtime >= (1.0/loop3_rate))
                {
                    run_loop3_code();
                    last_loop3_timer = ros::Time::now();
                }
            }
            
            mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
            if(mtime >= 0.1)
            {
                run_10Hz_code();
                last_10Hz_timer = ros::Time::now();
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
	diagnostic_status.Component = TIMING_NODE;

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
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
    pps01_sub = nh.subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback); 
    pps1_sub = nh.subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback); 
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    
    double max_rate = 0.0;
    std::string param_loop1_rate = node_name + "/loop1_rate";
    if(nh.getParam(param_loop1_rate,loop1_rate) == false)
    {
        logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
        run_loop1 = false;
    }
    else 
    { 
        last_loop1_timer = ros::Time::now();
        run_loop1 = true; 
        if(loop1_rate > max_rate) { max_rate = loop1_rate; }
    }
    
    std::string param_loop2_rate = node_name + "/loop2_rate";
    if(nh.getParam(param_loop2_rate,loop2_rate) == false)
    {
        logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
        run_loop2 = false;
    }
    else 
    { 
        last_loop2_timer = ros::Time::now();
        run_loop2 = true; 
        if(loop2_rate > max_rate) { max_rate = loop2_rate; }
    }
    
    std::string param_loop3_rate = node_name + "/loop3_rate";
    if(nh.getParam(param_loop3_rate,loop3_rate) == false)
    {
        logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
        run_loop3 = false;
    }
    else 
    { 
        last_loop3_timer = ros::Time::now();
        run_loop3 = true; 
        if(loop3_rate > max_rate) { max_rate = loop3_rate; }
    }
    ros_rate = max_rate * 50.0;
    if(ros_rate < 100.0) { ros_rate = 100.0; }
    char tempstr[512];
    sprintf(tempstr,"Running Node at Rate: %f",ros_rate);
    logger->log_notice(std::string(tempstr));
    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    process = new PoseNodeProcess;
	diagnostic_status = process->init(diagnostic_status,std::string(hostname));
	pan_angle = -M_PI/4.0;
	tilt_angle = -M_PI/4.0;
	jointstate_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
    //KF: Yaw
    {
		int output_count = 2; //n
		int measurement_count = 2; //m
		MatrixXd C(measurement_count,output_count);
		C(0,0) = 1.0;
		C(1,0) = 0.0;
		C(0,1) = 0.0;
		C(1,1) = 1.0;
		MatrixXd Phi(output_count,output_count);
		Phi(0,0) = 1.0;
		Phi(0,1) = 1.0/(double)(loop1_rate);
		Phi(1,0) = 0.0;
		Phi(1,1) = 1.0;
		MatrixXd Q(output_count,output_count);
		Q = MatrixXd::Zero(output_count,output_count);
		int QIndex = 0;
		while(QIndex < output_count)
		{
			double v;
			std::string param_topic = node_name +"/KF_Yaw_Q" + boost::lexical_cast<std::string>(QIndex);
			if(nh.getParam(param_topic,v) == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Didn't find %s. Exiting.",param_topic.c_str());
				logger->log_error(tempstr);
				return false;
			}
			else
			{
				Q(QIndex,QIndex) = v;
			}
			QIndex++;
    	}
		//cout << "KF: Yaw Q: " << Q << std::endl << std::endl;
		MatrixXd R(measurement_count,measurement_count);
		R = MatrixXd::Zero(measurement_count,measurement_count);
		int RIndex = 0;
		while(RIndex < output_count)
		{
			double v;
			std::string param_topic = node_name +"/KF_Yaw_R" + boost::lexical_cast<std::string>(RIndex);
			if(nh.getParam(param_topic,v) == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Didn't find %s. Exiting.",param_topic.c_str());
				logger->log_error(tempstr);
				return false;
			}
			else
			{
				R(RIndex,RIndex) = v;
			}
			RIndex++;
    	}
		//cout << "KF: Yaw R: " << R << std::endl << std::endl;
		process->set_kalmanfilter_properties("Yaw", output_count,measurement_count,C,Phi,Q,R);
    }
    //KF: Yawrate
    {
        int output_count = 1; //n
		int measurement_count = 1; //m
		MatrixXd C(measurement_count,output_count);
		C(0,0) = 1.0;
		MatrixXd Phi(output_count,output_count);
		Phi(0,0) = 1.0;
		MatrixXd Q(output_count,output_count);
		Q = MatrixXd::Zero(output_count,output_count);
		int QIndex = 0;
		while(QIndex < output_count)
		{
			double v;
			std::string param_topic = node_name +"/KF_Yaw_Q" + boost::lexical_cast<std::string>(QIndex);
			if(nh.getParam(param_topic,v) == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Didn't find %s. Exiting.",param_topic.c_str());
				logger->log_error(tempstr);
				return false;
			}
			else
			{
				Q(QIndex,QIndex) = v;
			}
			QIndex++;
    	}
		//cout << "KF: Yawrate Q: " << Q << std::endl << std::endl;
		MatrixXd R(measurement_count,measurement_count);
		R = MatrixXd::Zero(measurement_count,measurement_count);
		int RIndex = 0;
		while(RIndex < output_count)
		{
			double v;
			std::string param_topic = node_name +"/KF_Yaw_R" + boost::lexical_cast<std::string>(RIndex);
			if(nh.getParam(param_topic,v) == false)
			{
				char tempstr[255];
				sprintf(tempstr,"Didn't find %s. Exiting.",param_topic.c_str());
				logger->log_error(tempstr);
				return false;
			}
			else
			{
				R(RIndex,RIndex) = v;
			}
			RIndex++;
    	}
		//cout << "KF: Yaw R: " << R << std::endl << std::endl;
		process->set_kalmanfilter_properties("Yawrate", output_count,measurement_count,C,Phi,Q,R);
    }
    std::string param_attitude_topic = node_name + "/Attitude_topic";
    std::string attitude_topic;
    if(nh.getParam(param_attitude_topic,attitude_topic) == false)
    {
        logger->log_error("Missing parameter: Attitude_topic. Exiting.");
        return false;
    }
    attitude_sub = nh.subscribe<roscopter::Attitude>(attitude_topic,1,attitude_Callback);

    throttle_command_sub = nh.subscribe<std_msgs::Float32>("/throttle_command",1,throttle_command_Callback);
    steer_command_sub = nh.subscribe<std_msgs::Float32>("/steer_command",1,steer_command_Callback);
    pose_pub =  nh.advertise<icarus_rover_v2::pose>("/pose",1);
	
	std::string param_simulate = node_name  + "/Simulate";
	bool simulate;
	if(nh.getParam(param_simulate,simulate) == false)
	{
		logger->log_warn("Missing parameter: Simulate.  Using default: False");
		simulate = false;
	}
	if(simulate == true)
	{
		encoder_sub = nh.subscribe<icarus_rover_v2::encoder>("/sim/encoder",1,encoder_Callback);
		gps_sub = nh.subscribe<icarus_rover_v2::pose>("/sim/pose",1,gps_Callback);
	}
	else
	{
		encoder_sub = nh.subscribe<icarus_rover_v2::encoder>("/encoder",1,encoder_Callback);
		gps_sub = nh.subscribe<icarus_rover_v2::pose>("/pose",1,gps_Callback);
	}
	std::string param_compute_pose = node_name + "/Compute_Pose";
	bool compute_pose;
	if(nh.getParam(param_compute_pose,compute_pose) == false)
	{
		logger->log_warn("Missing parameter: Compute_Pose.  Will Compute Pose.");
		compute_pose = true;
	}
	if(compute_pose == false)
	{
		simpose_sub = nh.subscribe<icarus_rover_v2::pose>("/sim/pose",1,simpose_Callback);
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
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
