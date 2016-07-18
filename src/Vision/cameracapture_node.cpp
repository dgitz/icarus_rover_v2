#include "cameracapture_node.h"

//Start User Code: Functions
bool acquire_image(cv::VideoCapture cap)
{
    ros::Time start = ros::Time::now();
    cv_bridge::CvImage cv_image;
    cv::Mat frame;
    cap >> frame;
    cv_image.encoding = "rgb8";
    cv_image.header.stamp = ros::Time::now();
    cv_image.header.frame_id = "/world";
    cv_image.image = frame;
    image_pub.publish(cv_image.toImageMsg());
    ros::Duration how_long = ros::Time::now() - start;
    printf("Duration: %f\r\n",how_long.toSec());
    //char tempstr[30];
    //cv_bridge::CvImage cv_image;
    //sprintf(tempstr,"/home/robot/temp/test_%d.png",counter);
 
    //bool status = cv::imwrite(tempstr,image,compression_params);
    //counter++;
    return true;
}
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	return true;
}
bool run_slowrate_code()
{
 	if(device_initialized == true)
	{
		/*pid = get_pid();
		if(pid < 0)
		{
			logger->log_warn("Couldn't retrieve PID.");
		}
		else
		{
			if(check_resources(pid))
			{
				resource_pub.publish(resources_used);
			}
			else
			{
				logger->log_warn("Couldn't read resources used.");
			}
		}*/
	}
    
	//logger->log_debug("Running slow rate code.");
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = DEBUG;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");

	return true;
}

//End User Code: Functions

//Start Initialize Function
bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization, Parameters and Topics
    printf("Node name: %s\r\n",node_name.c_str());
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    pid = -1;
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
	char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
	 std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
		{
			logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
			return false;
		}
	//End Template Code: Initialization, Parameters and Topics

	//Start User Code: Initialization, Parameters and Topics
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = VISION_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

    image_pub = nh.advertise<sensor_msgs::Image>("/camera_image",1000);
    counter = 0;
	//End User Code: Initialization, Parameters and Topics
    logger->log_info("Initialized!");
    return true;
}
//End Initialize Function

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
    cv::VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);
    if(!capture.isOpened())  // check if we succeeded
    {
        printf("Can't initialize camera!\r\n");
    }
    else
    {
        printf("Camera working!\r\n");
    }
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
                acquire_image(capture);
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
    return 0;
}
//End Main Loop

//Start Template Code: Functions
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
int get_pid()
{
	int id = -1;
	std::string local_node_name;
	local_node_name = node_name.substr(1,node_name.size());
	std::string pid_filename;
	pid_filename = "/home/robot/logs/output/PID" + node_name;
	char tempstr[130];
	sprintf(tempstr,"ps aux | grep __name:=%s > %s",local_node_name.c_str(),pid_filename.c_str());
	system(tempstr);
	ifstream myfile;
	myfile.open(pid_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		//printf("Line:%s\r\n",line.c_str());
		std::size_t found = line.find("icarus_rover_v2/diagnostic_node");
		if(found != std::string::npos)
		{
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			id =  atoi(fields.at(1).c_str());
		}
	}
	else
	{
		id = -1;
	}
	myfile.close();
	//printf("ID: %d\r\n",id);
	//id = -1;
	return id;

}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	myDevice = newdevice;
	if(myDevice.DeviceName != "")
	{
		device_initialized = true;
	}
}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
bool check_resources(int procid)
{
	std::string resource_filename;
	resource_filename = "/home/robot/logs/output/RESOURCE/" + node_name;
	char tempstr[130];
	sprintf(tempstr,"top -bn1 | grep %d > %s",procid,resource_filename.c_str());
	//printf("Command: %s\r\n",tempstr);
	system(tempstr); //RAM used is column 6, in KB.  CPU used is column 8, in percentage.
	ifstream myfile;
	myfile.open(resource_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		std::vector<std::string> strs;
		boost::split(strs,line,boost::is_any_of(" "),boost::token_compress_on);
		resources_used.Node_Name = node_name;
		resources_used.PID = pid;
		resources_used.CPU_Perc = atoi(strs.at(8).c_str());
		resources_used.RAM_MB = atoi(strs.at(6).c_str())/1000.0;
		return true;
	}
	else
	{
		return false;
	}
	myfile.close();
	return false;
}
//End Template Code: Functions
