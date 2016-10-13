#include "cameracapture_node.h"
//Start Template Code: Firmware Definition
#define CAMERACAPTURENODE_MAJOR_RELEASE 1
#define CAMERACAPTURENODE_MINOR_RELEASE 1
#define CAMERACAPTURENODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
//Start User Code: Functions
void Edge_Detect_Threshold_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	logger->log_debug("Got edge detect threshold");
	edge_detect_threshold = msg->data;
}
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
    int bytes = frame.total() * frame.elemSize();
    if(bytes > 0)
    {
		raw_image_pub.publish(cv_image.toImageMsg());
    }
    char tempstr[128];
    sprintf(tempstr,"Image Data size: %d",bytes);
    logger->log_debug(tempstr);
    cv::Mat src_gray;
    cv::cvtColor(frame,src_gray,cv::COLOR_BGR2GRAY);
    cv::Mat edge_image = visionhelper->Detect_Edges(src_gray,edge_detect_threshold);

    cv_bridge::CvImage proc_image;
	proc_image.encoding = "mono8";
	proc_image.header.stamp = ros::Time::now();
	proc_image.header.frame_id = "/world";
	proc_image.image = edge_image;
	proc_image_pub.publish(proc_image.toImageMsg());
    //Edge_Detection(src_gray,0,0);
    return true;
}
bool Edge_Detection(cv::Mat gray_image,int,void*)
{
	cv::Mat det_edges;
	cv::blur(gray_image,det_edges,cv::Size(3,3));
	cv::Canny(det_edges,det_edges,edge_detect_threshold,edge_detect_threshold*3,3);
	cv_bridge::CvImage proc_image;
	proc_image.encoding = "mono8";
	proc_image.header.stamp = ros::Time::now();
	proc_image.header.frame_id = "/world";
	proc_image.image = det_edges;
	proc_image_pub.publish(proc_image.toImageMsg());
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
	acquire_image(capture);
	return true;
}
bool run_slowrate_code()
{
	if(device_initialized == true)
	{
		bool status = resourcemonitor->update();
		if(status == true)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
		else
		{
			logger->log_warn("Couldn't read resources used.");
		}
	}
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "cameracapture_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 8-Sep-2016";
	fw.Major_Release = CAMERACAPTURENODE_MAJOR_RELEASE;
	fw.Minor_Release = CAMERACAPTURENODE_MINOR_RELEASE;
	fw.Build_Number = CAMERACAPTURENODE_BUILD_NUMBER;
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
    capture.release();
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
	std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
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

	visionhelper = new VisionHelper();
	std::string param_image_width = node_name +"/image_width";
	if(nh.getParam(param_image_width,image_width) == false)
	{
		logger->log_fatal("Missing Parameter: image_width. Exiting");
		return false;
	}
	std::string param_image_height = node_name +"/image_height";
	if(nh.getParam(param_image_height,image_height) == false)
	{
		logger->log_fatal("Missing Parameter: image_height. Exiting");
		return false;
	}
	int video_device;
	std::string param_video_device = node_name +"/video_device";
	if(nh.getParam(param_video_device,video_device) == false)
	{
		logger->log_fatal("Missing Parameter: video_device. Exiting");
		return false;
	}
	std::string rawimage_topic = "/" + node_name + "/raw_image";
    raw_image_pub = nh.advertise<sensor_msgs::Image>(rawimage_topic,1000);
    std::string procimage_topic = "/" + node_name + "/proc_image";
	proc_image_pub = nh.advertise<sensor_msgs::Image>(procimage_topic,1000);
    counter = 0;
    edge_detect_threshold = 100;
    std::string edge_detect_topic = "/" + node_name +"/edge_detect_threshold";
    edge_threshold_sub = nh.subscribe<std_msgs::UInt8>(edge_detect_topic,1000,Edge_Detect_Threshold_Callback);
    capture.open(video_device);
    printf("here\n");
	capture.set(CV_CAP_PROP_FRAME_WIDTH, image_width);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //CV_IMWRITE_PNG_COMPRESSION
	compression_params.push_back(3);  //3
 	if(!capture.isOpened())  // check if we succeeded
	{
		logger->log_fatal("Can't initialize camera. Exiting.");
		return false;
	}
	else
	{
	   logger->log_info("Camera working!");
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
	myDevice = newdevice;
	if(myDevice.DeviceName != "")
	{
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
//End Template Code: Functions
